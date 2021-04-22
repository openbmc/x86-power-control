/*
// Copyright (c) 2018-2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/
#include "i2c.hpp"

#include <sys/sysinfo.h>
#include <systemd/sd-journal.h>

#include <boost/asio/io_service.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <gpiod.hpp>
#include <nlohmann/json.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <filesystem>
#include <fstream>
#include <string_view>

namespace power_control
{
static boost::asio::io_service io;
std::shared_ptr<sdbusplus::asio::connection> conn;

static std::string node = "0";

static std::string powerOutName;
static std::string powerOkName;
static std::string resetOutName;
static std::string nmiOutName;
static std::string sioPwrGoodName;
static std::string sioOnControlName;
static std::string sioS5Name;
static std::string postCompleteName;
static std::string powerButtonName;
static std::string resetButtonName;
static std::string idButtonName;
static std::string nmiButtonName;

static std::shared_ptr<sdbusplus::asio::dbus_interface> hostIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> chassisIface;
#ifdef CHASSIS_SYSTEM_RESET
static std::shared_ptr<sdbusplus::asio::dbus_interface> chassisSysIface;
#endif
static std::shared_ptr<sdbusplus::asio::dbus_interface> powerButtonIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> resetButtonIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> nmiButtonIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> osIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> idButtonIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> nmiOutIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> restartCauseIface;

static gpiod::line powerButtonMask;
static gpiod::line resetButtonMask;
static bool nmiButtonMasked = false;

static int powerPulseTimeMs = 200;
static int forceOffPulseTimeMs = 15000;
static int resetPulseTimeMs = 500;
static int powerCycleTimeMs = 5000;
static int sioPowerGoodWatchdogTimeMs = 1000;
static int psPowerOKWatchdogTimeMs = 8000;
static int gracefulPowerOffTimeS = 5 * 60;
static int warmResetCheckTimeMs = 500;
static int powerOffSaveTimeMs = 7000;

const static std::filesystem::path powerControlDir = "/var/lib/power-control";
const static constexpr std::string_view powerStateFile = "power-state";

static bool nmiEnabled = true;
static bool sioEnabled = true;

// Timers
// Time holding GPIOs asserted
static boost::asio::steady_timer gpioAssertTimer(io);
// Time between off and on during a power cycle
static boost::asio::steady_timer powerCycleTimer(io);
// Time OS gracefully powering off
static boost::asio::steady_timer gracefulPowerOffTimer(io);
// Time the warm reset check
static boost::asio::steady_timer warmResetCheckTimer(io);
// Time power supply power OK assertion on power-on
static boost::asio::steady_timer psPowerOKWatchdogTimer(io);
// Time SIO power good assertion on power-on
static boost::asio::steady_timer sioPowerGoodWatchdogTimer(io);
// Time power-off state save for power loss tracking
static boost::asio::steady_timer powerStateSaveTimer(io);
// POH timer
static boost::asio::steady_timer pohCounterTimer(io);
// Time when to allow restart cause updates
static boost::asio::steady_timer restartCauseTimer(io);

// GPIO Lines and Event Descriptors
static gpiod::line psPowerOKLine;
static boost::asio::posix::stream_descriptor psPowerOKEvent(io);
static gpiod::line sioPowerGoodLine;
static boost::asio::posix::stream_descriptor sioPowerGoodEvent(io);
static gpiod::line sioOnControlLine;
static boost::asio::posix::stream_descriptor sioOnControlEvent(io);
static gpiod::line sioS5Line;
static boost::asio::posix::stream_descriptor sioS5Event(io);
static gpiod::line powerButtonLine;
static boost::asio::posix::stream_descriptor powerButtonEvent(io);
static gpiod::line resetButtonLine;
static boost::asio::posix::stream_descriptor resetButtonEvent(io);
static gpiod::line nmiButtonLine;
static boost::asio::posix::stream_descriptor nmiButtonEvent(io);
static gpiod::line idButtonLine;
static boost::asio::posix::stream_descriptor idButtonEvent(io);
static gpiod::line postCompleteLine;
static boost::asio::posix::stream_descriptor postCompleteEvent(io);
static gpiod::line nmiOutLine;

static constexpr uint8_t beepPowerFail = 8;

static void beep(const uint8_t& beepPriority)
{
    std::string logMsg = "Beep with priority: " + std::to_string(beepPriority);
    phosphor::logging::log<phosphor::logging::level::INFO>(logMsg.c_str());

    conn->async_method_call(
        [](boost::system::error_code ec) {
            if (ec)
            {
                std::string errMsg =
                    "beep returned error with async_method_call (ec = " +
                    ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
                return;
            }
        },
        "xyz.openbmc_project.BeepCode", "/xyz/openbmc_project/BeepCode",
        "xyz.openbmc_project.BeepCode", "Beep", uint8_t(beepPriority));
}

enum class PowerState
{
    on,
    waitForPSPowerOK,
    waitForSIOPowerGood,
    off,
    transitionToOff,
    gracefulTransitionToOff,
    cycleOff,
    transitionToCycleOff,
    gracefulTransitionToCycleOff,
    checkForWarmReset,
};
static PowerState powerState;
static std::string getPowerStateName(PowerState state)
{
    switch (state)
    {
        case PowerState::on:
            return "On";
            break;
        case PowerState::waitForPSPowerOK:
            return "Wait for Power Supply Power OK";
            break;
        case PowerState::waitForSIOPowerGood:
            return "Wait for SIO Power Good";
            break;
        case PowerState::off:
            return "Off";
            break;
        case PowerState::transitionToOff:
            return "Transition to Off";
            break;
        case PowerState::gracefulTransitionToOff:
            return "Graceful Transition to Off";
            break;
        case PowerState::cycleOff:
            return "Power Cycle Off";
            break;
        case PowerState::transitionToCycleOff:
            return "Transition to Power Cycle Off";
            break;
        case PowerState::gracefulTransitionToCycleOff:
            return "Graceful Transition to Power Cycle Off";
            break;
        case PowerState::checkForWarmReset:
            return "Check for Warm Reset";
            break;
        default:
            return "unknown state: " + std::to_string(static_cast<int>(state));
            break;
    }
}
static void logStateTransition(const PowerState state)
{
    std::string logMsg =
        "Host0: Moving to \"" + getPowerStateName(state) + "\" state";
    phosphor::logging::log<phosphor::logging::level::INFO>(
        logMsg.c_str(),
        phosphor::logging::entry("STATE=%s", getPowerStateName(state).c_str()),
        phosphor::logging::entry("HOST=0"));
}

enum class Event
{
    psPowerOKAssert,
    psPowerOKDeAssert,
    sioPowerGoodAssert,
    sioPowerGoodDeAssert,
    sioS5Assert,
    sioS5DeAssert,
    pltRstAssert,
    pltRstDeAssert,
    postCompleteAssert,
    postCompleteDeAssert,
    powerButtonPressed,
    resetButtonPressed,
    powerCycleTimerExpired,
    psPowerOKWatchdogTimerExpired,
    sioPowerGoodWatchdogTimerExpired,
    gracefulPowerOffTimerExpired,
    powerOnRequest,
    powerOffRequest,
    powerCycleRequest,
    resetRequest,
    gracefulPowerOffRequest,
    gracefulPowerCycleRequest,
    warmResetDetected,
};
static std::string getEventName(Event event)
{
    switch (event)
    {
        case Event::psPowerOKAssert:
            return "power supply power OK assert";
            break;
        case Event::psPowerOKDeAssert:
            return "power supply power OK de-assert";
            break;
        case Event::sioPowerGoodAssert:
            return "SIO power good assert";
            break;
        case Event::sioPowerGoodDeAssert:
            return "SIO power good de-assert";
            break;
        case Event::sioS5Assert:
            return "SIO S5 assert";
            break;
        case Event::sioS5DeAssert:
            return "SIO S5 de-assert";
            break;
        case Event::pltRstAssert:
            return "PLT_RST assert";
            break;
        case Event::pltRstDeAssert:
            return "PLT_RST de-assert";
            break;
        case Event::postCompleteAssert:
            return "POST Complete assert";
            break;
        case Event::postCompleteDeAssert:
            return "POST Complete de-assert";
            break;
        case Event::powerButtonPressed:
            return "power button pressed";
            break;
        case Event::resetButtonPressed:
            return "reset button pressed";
            break;
        case Event::powerCycleTimerExpired:
            return "power cycle timer expired";
            break;
        case Event::psPowerOKWatchdogTimerExpired:
            return "power supply power OK watchdog timer expired";
            break;
        case Event::sioPowerGoodWatchdogTimerExpired:
            return "SIO power good watchdog timer expired";
            break;
        case Event::gracefulPowerOffTimerExpired:
            return "graceful power-off timer expired";
            break;
        case Event::powerOnRequest:
            return "power-on request";
            break;
        case Event::powerOffRequest:
            return "power-off request";
            break;
        case Event::powerCycleRequest:
            return "power-cycle request";
            break;
        case Event::resetRequest:
            return "reset request";
            break;
        case Event::gracefulPowerOffRequest:
            return "graceful power-off request";
            break;
        case Event::gracefulPowerCycleRequest:
            return "graceful power-cycle request";
            break;
        case Event::warmResetDetected:
            return "warm reset detected";
            break;
        default:
            return "unknown event: " + std::to_string(static_cast<int>(event));
            break;
    }
}
static void logEvent(const std::string_view stateHandler, const Event event)
{
    std::string logMsg{stateHandler};
    logMsg += ": " + getEventName(event) + " event received";
    phosphor::logging::log<phosphor::logging::level::INFO>(
        logMsg.c_str(),
        phosphor::logging::entry("EVENT=%s", getEventName(event).c_str()));
}

// Power state handlers
static void powerStateOn(const Event event);
static void powerStateWaitForPSPowerOK(const Event event);
static void powerStateWaitForSIOPowerGood(const Event event);
static void powerStateOff(const Event event);
static void powerStateTransitionToOff(const Event event);
static void powerStateGracefulTransitionToOff(const Event event);
static void powerStateCycleOff(const Event event);
static void powerStateTransitionToCycleOff(const Event event);
static void powerStateGracefulTransitionToCycleOff(const Event event);
static void powerStateCheckForWarmReset(const Event event);

static std::function<void(const Event)> getPowerStateHandler(PowerState state)
{
    switch (state)
    {
        case PowerState::on:
            return powerStateOn;
            break;
        case PowerState::waitForPSPowerOK:
            return powerStateWaitForPSPowerOK;
            break;
        case PowerState::waitForSIOPowerGood:
            return powerStateWaitForSIOPowerGood;
            break;
        case PowerState::off:
            return powerStateOff;
            break;
        case PowerState::transitionToOff:
            return powerStateTransitionToOff;
            break;
        case PowerState::gracefulTransitionToOff:
            return powerStateGracefulTransitionToOff;
            break;
        case PowerState::cycleOff:
            return powerStateCycleOff;
            break;
        case PowerState::transitionToCycleOff:
            return powerStateTransitionToCycleOff;
            break;
        case PowerState::gracefulTransitionToCycleOff:
            return powerStateGracefulTransitionToCycleOff;
            break;
        case PowerState::checkForWarmReset:
            return powerStateCheckForWarmReset;
            break;
        default:
            return nullptr;
            break;
    }
};

static void sendPowerControlEvent(const Event event)
{
    std::function<void(const Event)> handler = getPowerStateHandler(powerState);
    if (handler == nullptr)
    {
        std::string errMsg = "Failed to find handler for power state: " +
                             std::to_string(static_cast<int>(powerState));
        phosphor::logging::log<phosphor::logging::level::INFO>(errMsg.c_str());
        return;
    }
    handler(event);
}

static uint64_t getCurrentTimeMs()
{
    struct timespec time = {};

    if (clock_gettime(CLOCK_REALTIME, &time) < 0)
    {
        return 0;
    }
    uint64_t currentTimeMs = static_cast<uint64_t>(time.tv_sec) * 1000;
    currentTimeMs += static_cast<uint64_t>(time.tv_nsec) / 1000 / 1000;

    return currentTimeMs;
}

static constexpr std::string_view getHostState(const PowerState state)
{
    switch (state)
    {
        case PowerState::on:
        case PowerState::gracefulTransitionToOff:
        case PowerState::gracefulTransitionToCycleOff:
            return "xyz.openbmc_project.State.Host.HostState.Running";
            break;
        case PowerState::waitForPSPowerOK:
        case PowerState::waitForSIOPowerGood:
        case PowerState::off:
        case PowerState::transitionToOff:
        case PowerState::transitionToCycleOff:
        case PowerState::cycleOff:
        case PowerState::checkForWarmReset:
            return "xyz.openbmc_project.State.Host.HostState.Off";
            break;
        default:
            return "";
            break;
    }
};
static constexpr std::string_view getChassisState(const PowerState state)
{
    switch (state)
    {
        case PowerState::on:
        case PowerState::transitionToOff:
        case PowerState::gracefulTransitionToOff:
        case PowerState::transitionToCycleOff:
        case PowerState::gracefulTransitionToCycleOff:
        case PowerState::checkForWarmReset:
            return "xyz.openbmc_project.State.Chassis.PowerState.On";
            break;
        case PowerState::waitForPSPowerOK:
        case PowerState::waitForSIOPowerGood:
        case PowerState::off:
        case PowerState::cycleOff:
            return "xyz.openbmc_project.State.Chassis.PowerState.Off";
            break;
        default:
            return "";
            break;
    }
};
static void savePowerState(const PowerState state)
{
    powerStateSaveTimer.expires_after(
        std::chrono::milliseconds(powerOffSaveTimeMs));
    powerStateSaveTimer.async_wait([state](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::string errMsg =
                    "Power-state save async_wait failed: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
            }
            return;
        }
        std::ofstream powerStateStream(powerControlDir / powerStateFile);
        powerStateStream << getChassisState(state);
    });
}
static void setPowerState(const PowerState state)
{
    powerState = state;
    logStateTransition(state);

    hostIface->set_property("CurrentHostState",
                            std::string(getHostState(powerState)));

    chassisIface->set_property("CurrentPowerState",
                               std::string(getChassisState(powerState)));
    chassisIface->set_property("LastStateChangeTime", getCurrentTimeMs());

    // Save the power state for the restore policy
    savePowerState(state);
}

enum class RestartCause
{
    command,
    resetButton,
    powerButton,
    watchdog,
    powerPolicyOn,
    powerPolicyRestore,
    softReset,
};
static boost::container::flat_set<RestartCause> causeSet;
static std::string getRestartCause(RestartCause cause)
{
    switch (cause)
    {
        case RestartCause::command:
            return "xyz.openbmc_project.State.Host.RestartCause.IpmiCommand";
            break;
        case RestartCause::resetButton:
            return "xyz.openbmc_project.State.Host.RestartCause.ResetButton";
            break;
        case RestartCause::powerButton:
            return "xyz.openbmc_project.State.Host.RestartCause.PowerButton";
            break;
        case RestartCause::watchdog:
            return "xyz.openbmc_project.State.Host.RestartCause.WatchdogTimer";
            break;
        case RestartCause::powerPolicyOn:
            return "xyz.openbmc_project.State.Host.RestartCause."
                   "PowerPolicyAlwaysOn";
            break;
        case RestartCause::powerPolicyRestore:
            return "xyz.openbmc_project.State.Host.RestartCause."
                   "PowerPolicyPreviousState";
            break;
        case RestartCause::softReset:
            return "xyz.openbmc_project.State.Host.RestartCause.SoftReset";
            break;
        default:
            return "xyz.openbmc_project.State.Host.RestartCause.Unknown";
            break;
    }
}
static void addRestartCause(const RestartCause cause)
{
    // Add this to the set of causes for this restart
    causeSet.insert(cause);
}
static void clearRestartCause()
{
    // Clear the set for the next restart
    causeSet.clear();
}
static void setRestartCauseProperty(const std::string& cause)
{
    std::string logMsg = "RestartCause set to " + cause;
    phosphor::logging::log<phosphor::logging::level::INFO>(logMsg.c_str());
    restartCauseIface->set_property("RestartCause", cause);
}

static void resetACBootProperty()
{
    if ((causeSet.contains(RestartCause::command)) ||
        (causeSet.contains(RestartCause::softReset)))
    {
        conn->async_method_call(
            [](boost::system::error_code ec) {
                if (ec)
                {
                    phosphor::logging::log<phosphor::logging::level::ERR>(
                        "failed to reset ACBoot property");
                }
            },
            "xyz.openbmc_project.Settings",
            "/xyz/openbmc_project/control/host0/ac_boot",
            "org.freedesktop.DBus.Properties", "Set",
            "xyz.openbmc_project.Common.ACBoot", "ACBoot",
            std::variant<std::string>{"False"});
    }
}

static void setRestartCause()
{
    // Determine the actual restart cause based on the set of causes
    std::string restartCause =
        "xyz.openbmc_project.State.Host.RestartCause.Unknown";
    if (causeSet.contains(RestartCause::watchdog))
    {
        restartCause = getRestartCause(RestartCause::watchdog);
    }
    else if (causeSet.contains(RestartCause::command))
    {
        restartCause = getRestartCause(RestartCause::command);
    }
    else if (causeSet.contains(RestartCause::resetButton))
    {
        restartCause = getRestartCause(RestartCause::resetButton);
    }
    else if (causeSet.contains(RestartCause::powerButton))
    {
        restartCause = getRestartCause(RestartCause::powerButton);
    }
    else if (causeSet.contains(RestartCause::powerPolicyOn))
    {
        restartCause = getRestartCause(RestartCause::powerPolicyOn);
    }
    else if (causeSet.contains(RestartCause::powerPolicyRestore))
    {
        restartCause = getRestartCause(RestartCause::powerPolicyRestore);
    }
    else if (causeSet.contains(RestartCause::softReset))
    {
        restartCause = getRestartCause(RestartCause::softReset);
    }

    setRestartCauseProperty(restartCause);
}

static void systemPowerGoodFailedLog()
{
    sd_journal_send(
        "MESSAGE=PowerControl: system power good failed to assert (VR failure)",
        "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
        "OpenBMC.0.1.SystemPowerGoodFailed", "REDFISH_MESSAGE_ARGS=%d",
        sioPowerGoodWatchdogTimeMs, NULL);
}

static void psPowerOKFailedLog()
{
    sd_journal_send(
        "MESSAGE=PowerControl: power supply power good failed to assert",
        "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
        "OpenBMC.0.1.PowerSupplyPowerGoodFailed", "REDFISH_MESSAGE_ARGS=%d",
        psPowerOKWatchdogTimeMs, NULL);
}

static void powerRestorePolicyLog()
{
    sd_journal_send("MESSAGE=PowerControl: power restore policy applied",
                    "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.PowerRestorePolicyApplied", NULL);
}

static void powerButtonPressLog()
{
    sd_journal_send("MESSAGE=PowerControl: power button pressed", "PRIORITY=%i",
                    LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.PowerButtonPressed", NULL);
}

static void resetButtonPressLog()
{
    sd_journal_send("MESSAGE=PowerControl: reset button pressed", "PRIORITY=%i",
                    LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.ResetButtonPressed", NULL);
}

static void nmiButtonPressLog()
{
    sd_journal_send("MESSAGE=PowerControl: NMI button pressed", "PRIORITY=%i",
                    LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.NMIButtonPressed", NULL);
}

static void nmiDiagIntLog()
{
    sd_journal_send("MESSAGE=PowerControl: NMI Diagnostic Interrupt",
                    "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.NMIDiagnosticInterrupt", NULL);
}

static int initializePowerStateStorage()
{
    // create the power control directory if it doesn't exist
    std::error_code ec;
    if (!(std::filesystem::create_directories(powerControlDir, ec)))
    {
        if (ec.value() != 0)
        {
            std::string errMsg =
                "failed to create " + powerControlDir.string() + ec.message();
            phosphor::logging::log<phosphor::logging::level::ERR>(
                errMsg.c_str());
            return -1;
        }
    }
    // Create the power state file if it doesn't exist
    if (!std::filesystem::exists(powerControlDir / powerStateFile))
    {
        std::ofstream powerStateStream(powerControlDir / powerStateFile);
        powerStateStream << getChassisState(powerState);
    }
    return 0;
}

static bool wasPowerDropped()
{
    std::ifstream powerStateStream(powerControlDir / powerStateFile);
    if (!powerStateStream.is_open())
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Failed to open power state file");
        return false;
    }

    std::string state;
    std::getline(powerStateStream, state);
    return state == "xyz.openbmc_project.State.Chassis.PowerState.On";
}

static void invokePowerRestorePolicy(const std::string& policy)
{
    // Async events may call this twice, but we only want to run once
    static bool policyInvoked = false;
    if (policyInvoked)
    {
        return;
    }
    policyInvoked = true;

    std::string logMsg = "Power restore delay expired, invoking " + policy;
    phosphor::logging::log<phosphor::logging::level::INFO>(logMsg.c_str());
    if (policy ==
        "xyz.openbmc_project.Control.Power.RestorePolicy.Policy.AlwaysOn")
    {
        sendPowerControlEvent(Event::powerOnRequest);
        setRestartCauseProperty(getRestartCause(RestartCause::powerPolicyOn));
    }
    else if (policy == "xyz.openbmc_project.Control.Power.RestorePolicy."
                       "Policy.Restore")
    {
        if (wasPowerDropped())
        {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "Power was dropped, restoring Host On state");
            sendPowerControlEvent(Event::powerOnRequest);
            setRestartCauseProperty(
                getRestartCause(RestartCause::powerPolicyRestore));
        }
        else
        {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No power drop, restoring Host Off state");
        }
    }
    // We're done with the previous power state for the restore policy, so store
    // the current state
    savePowerState(powerState);
}

static void powerRestorePolicyDelay(int delay)
{
    // Async events may call this twice, but we only want to run once
    static bool delayStarted = false;
    if (delayStarted)
    {
        return;
    }
    delayStarted = true;
    // Calculate the delay from now to meet the requested delay
    // Subtract the approximate uboot time
    static constexpr const int ubootSeconds = 20;
    delay -= ubootSeconds;
    // Subtract the time since boot
    struct sysinfo info = {};
    if (sysinfo(&info) == 0)
    {
        delay -= info.uptime;
    }
    // 0 is the minimum delay
    delay = std::max(delay, 0);

    static boost::asio::steady_timer powerRestorePolicyTimer(io);
    powerRestorePolicyTimer.expires_after(std::chrono::seconds(delay));
    std::string logMsg =
        "Power restore delay of " + std::to_string(delay) + " seconds started";
    phosphor::logging::log<phosphor::logging::level::INFO>(logMsg.c_str());
    powerRestorePolicyTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::string errMsg =
                    "power restore policy async_wait failed: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
            }
            return;
        }
        // Get Power Restore Policy
        // In case PowerRestorePolicy is not available, set a match for it
        static std::unique_ptr<sdbusplus::bus::match::match>
            powerRestorePolicyMatch = std::make_unique<
                sdbusplus::bus::match::match>(
                *conn,
                "type='signal',interface='org.freedesktop.DBus.Properties',"
                "member='PropertiesChanged',arg0namespace='xyz.openbmc_"
                "project.Control.Power.RestorePolicy'",
                [](sdbusplus::message::message& msg) {
                    std::string interfaceName;
                    boost::container::flat_map<std::string,
                                               std::variant<std::string>>
                        propertiesChanged;
                    std::string policy;
                    try
                    {
                        msg.read(interfaceName, propertiesChanged);
                        policy = std::get<std::string>(
                            propertiesChanged.begin()->second);
                    }
                    catch (std::exception& e)
                    {
                        phosphor::logging::log<phosphor::logging::level::ERR>(
                            "Unable to read power restore policy value");
                        powerRestorePolicyMatch.reset();
                        return;
                    }
                    invokePowerRestorePolicy(policy);
                    powerRestorePolicyMatch.reset();
                });

        // Check if it's already on DBus
        conn->async_method_call(
            [](boost::system::error_code ec,
               const std::variant<std::string>& policyProperty) {
                if (ec)
                {
                    return;
                }
                powerRestorePolicyMatch.reset();
                const std::string* policy =
                    std::get_if<std::string>(&policyProperty);
                if (policy == nullptr)
                {
                    phosphor::logging::log<phosphor::logging::level::ERR>(
                        "Unable to read power restore policy value");
                    return;
                }
                invokePowerRestorePolicy(*policy);
            },
            "xyz.openbmc_project.Settings",
            "/xyz/openbmc_project/control/host0/power_restore_policy",
            "org.freedesktop.DBus.Properties", "Get",
            "xyz.openbmc_project.Control.Power.RestorePolicy",
            "PowerRestorePolicy");
    });
}

static void powerRestorePolicyStart()
{
    phosphor::logging::log<phosphor::logging::level::INFO>(
        "Power restore policy started");
    powerRestorePolicyLog();

    // Get the desired delay time
    // In case PowerRestoreDelay is not available, set a match for it
    static std::unique_ptr<sdbusplus::bus::match::match>
        powerRestoreDelayMatch = std::make_unique<sdbusplus::bus::match::match>(
            *conn,
            "type='signal',interface='org.freedesktop.DBus.Properties',member='"
            "PropertiesChanged',arg0namespace='xyz.openbmc_project.Control."
            "Power.RestoreDelay'",
            [](sdbusplus::message::message& msg) {
                std::string interfaceName;
                boost::container::flat_map<std::string, std::variant<uint16_t>>
                    propertiesChanged;
                int delay = 0;
                try
                {
                    msg.read(interfaceName, propertiesChanged);
                    delay =
                        std::get<uint16_t>(propertiesChanged.begin()->second);
                }
                catch (std::exception& e)
                {
                    phosphor::logging::log<phosphor::logging::level::ERR>(
                        "Unable to read power restore delay value");
                    powerRestoreDelayMatch.reset();
                    return;
                }
                powerRestorePolicyDelay(delay);
                powerRestoreDelayMatch.reset();
            });

    // Check if it's already on DBus
    conn->async_method_call(
        [](boost::system::error_code ec,
           const std::variant<uint16_t>& delayProperty) {
            if (ec)
            {
                return;
            }
            powerRestoreDelayMatch.reset();
            const uint16_t* delay = std::get_if<uint16_t>(&delayProperty);
            if (delay == nullptr)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "Unable to read power restore delay value");
                return;
            }
            powerRestorePolicyDelay(*delay);
        },
        "xyz.openbmc_project.Settings",
        "/xyz/openbmc_project/control/power_restore_delay",
        "org.freedesktop.DBus.Properties", "Get",
        "xyz.openbmc_project.Control.Power.RestoreDelay", "PowerRestoreDelay");
}

static void powerRestorePolicyCheck()
{
    // In case ACBoot is not available, set a match for it
    static std::unique_ptr<sdbusplus::bus::match::match> acBootMatch =
        std::make_unique<sdbusplus::bus::match::match>(
            *conn,
            "type='signal',interface='org.freedesktop.DBus.Properties',member='"
            "PropertiesChanged',arg0namespace='xyz.openbmc_project.Common."
            "ACBoot'",
            [](sdbusplus::message::message& msg) {
                std::string interfaceName;
                boost::container::flat_map<std::string,
                                           std::variant<std::string>>
                    propertiesChanged;
                std::string acBoot;
                try
                {
                    msg.read(interfaceName, propertiesChanged);
                    acBoot = std::get<std::string>(
                        propertiesChanged.begin()->second);
                }
                catch (std::exception& e)
                {
                    phosphor::logging::log<phosphor::logging::level::ERR>(
                        "Unable to read AC Boot status");
                    acBootMatch.reset();
                    return;
                }
                if (acBoot == "Unknown")
                {
                    return;
                }
                if (acBoot == "True")
                {
                    // Start the Power Restore policy
                    powerRestorePolicyStart();
                }
                acBootMatch.reset();
            });

    // Check if it's already on DBus
    conn->async_method_call(
        [](boost::system::error_code ec,
           const std::variant<std::string>& acBootProperty) {
            if (ec)
            {
                return;
            }
            const std::string* acBoot =
                std::get_if<std::string>(&acBootProperty);
            if (acBoot == nullptr)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "Unable to read AC Boot status");
                return;
            }
            if (*acBoot == "Unknown")
            {
                return;
            }
            if (*acBoot == "True")
            {
                // Start the Power Restore policy
                powerRestorePolicyStart();
            }
            acBootMatch.reset();
        },
        "xyz.openbmc_project.Settings",
        "/xyz/openbmc_project/control/host0/ac_boot",
        "org.freedesktop.DBus.Properties", "Get",
        "xyz.openbmc_project.Common.ACBoot", "ACBoot");
}

static bool requestGPIOEvents(
    const std::string& name, const std::function<void()>& handler,
    gpiod::line& gpioLine,
    boost::asio::posix::stream_descriptor& gpioEventDescriptor)
{
    // Find the GPIO line
    gpioLine = gpiod::find_line(name);
    if (!gpioLine)
    {
        std::string errMsg = "Failed to find the " + name + " line";
        phosphor::logging::log<phosphor::logging::level::ERR>(errMsg.c_str());
        return false;
    }

    try
    {
        gpioLine.request(
            {"power-control", gpiod::line_request::EVENT_BOTH_EDGES});
    }
    catch (std::exception&)
    {
        std::string errMsg = "Failed to request events for " + name;
        phosphor::logging::log<phosphor::logging::level::ERR>(errMsg.c_str());
        return false;
    }

    int gpioLineFd = gpioLine.event_get_fd();
    if (gpioLineFd < 0)
    {
        std::string errMsg = "Failed to name " + name + " fd";
        phosphor::logging::log<phosphor::logging::level::ERR>(errMsg.c_str());
        return false;
    }

    gpioEventDescriptor.assign(gpioLineFd);

    gpioEventDescriptor.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [&name, handler](const boost::system::error_code ec) {
            if (ec)
            {
                std::string errMsg =
                    name + " fd handler error: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
                // TODO: throw here to force power-control to restart?
                return;
            }
            handler();
        });
    return true;
}

static bool setGPIOOutput(const std::string& name, const int value,
                          gpiod::line& gpioLine)
{
    // Find the GPIO line
    gpioLine = gpiod::find_line(name);
    if (!gpioLine)
    {
        std::string errMsg = "Failed to find the " + name + " line";
        phosphor::logging::log<phosphor::logging::level::ERR>(errMsg.c_str());
        return false;
    }

    // Request GPIO output to specified value
    try
    {
        gpioLine.request({__FUNCTION__, gpiod::line_request::DIRECTION_OUTPUT},
                         value);
    }
    catch (std::exception&)
    {
        std::string errMsg = "Failed to request " + name + " output";
        phosphor::logging::log<phosphor::logging::level::ERR>(errMsg.c_str());
        return false;
    }

    std::string logMsg = name + " set to " + std::to_string(value);
    phosphor::logging::log<phosphor::logging::level::INFO>(logMsg.c_str());
    return true;
}

static int setMaskedGPIOOutputForMs(gpiod::line& maskedGPIOLine,
                                    const std::string& name, const int value,
                                    const int durationMs)
{
    // Set the masked GPIO line to the specified value
    maskedGPIOLine.set_value(value);
    std::string logMsg = name + " set to " + std::to_string(value);
    phosphor::logging::log<phosphor::logging::level::INFO>(logMsg.c_str());
    gpioAssertTimer.expires_after(std::chrono::milliseconds(durationMs));
    gpioAssertTimer.async_wait([maskedGPIOLine, value,
                                name](const boost::system::error_code ec) {
        // Set the masked GPIO line back to the opposite value
        maskedGPIOLine.set_value(!value);
        std::string logMsg = name + " released";
        phosphor::logging::log<phosphor::logging::level::INFO>(logMsg.c_str());
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::string errMsg =
                    name + " async_wait failed: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
            }
        }
    });
    return 0;
}

static int setGPIOOutputForMs(const std::string& name, const int value,
                              const int durationMs)
{
    // If the requested GPIO is masked, use the mask line to set the output
    if (powerButtonMask && name == power_control::powerOutName)
    {
        return setMaskedGPIOOutputForMs(powerButtonMask, name, value,
                                        durationMs);
    }
    if (resetButtonMask && name == power_control::resetOutName)
    {
        return setMaskedGPIOOutputForMs(resetButtonMask, name, value,
                                        durationMs);
    }

    // No mask set, so request and set the GPIO normally
    gpiod::line gpioLine;
    if (!setGPIOOutput(name, value, gpioLine))
    {
        return -1;
    }
    gpioAssertTimer.expires_after(std::chrono::milliseconds(durationMs));
    gpioAssertTimer.async_wait([gpioLine, value,
                                name](const boost::system::error_code ec) {
        // Set the GPIO line back to the opposite value
        gpioLine.set_value(!value);
        std::string logMsg = name + " released";
        phosphor::logging::log<phosphor::logging::level::INFO>(logMsg.c_str());
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::string errMsg =
                    name + " async_wait failed: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
            }
        }
    });
    return 0;
}

static void powerOn()
{
    setGPIOOutputForMs(power_control::powerOutName, 0, powerPulseTimeMs);
}

static void gracefulPowerOff()
{
    setGPIOOutputForMs(power_control::powerOutName, 0, powerPulseTimeMs);
}

static void forcePowerOff()
{
    if (setGPIOOutputForMs(power_control::powerOutName, 0,
                           forceOffPulseTimeMs) < 0)
    {
        return;
    }

    // If the force off timer expires, then the PCH power-button override
    // failed, so attempt the Unconditional Powerdown SMBus command.
    gpioAssertTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::string errMsg =
                    "Force power off async_wait failed: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
            }
            return;
        }

        phosphor::logging::log<phosphor::logging::level::INFO>(
            "PCH Power-button override failed. Issuing Unconditional Powerdown "
            "SMBus command.");
        const static constexpr size_t pchDevBusAddress = 3;
        const static constexpr size_t pchDevSlaveAddress = 0x44;
        const static constexpr size_t pchCmdReg = 0;
        const static constexpr size_t pchPowerDownCmd = 0x02;
        if (i2cSet(pchDevBusAddress, pchDevSlaveAddress, pchCmdReg,
                   pchPowerDownCmd) < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Unconditional Powerdown command failed! Not sure what to do "
                "now.");
        }
    });
}

static void reset()
{
    setGPIOOutputForMs(power_control::resetOutName, 0, resetPulseTimeMs);
}

static void gracefulPowerOffTimerStart()
{
    phosphor::logging::log<phosphor::logging::level::INFO>(
        "Graceful power-off timer started");
    gracefulPowerOffTimer.expires_after(
        std::chrono::seconds(gracefulPowerOffTimeS));
    gracefulPowerOffTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::string errMsg =
                    "Graceful power-off async_wait failed: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
            }
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "Graceful power-off timer canceled");
            return;
        }
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "Graceful power-off timer completed");
        sendPowerControlEvent(Event::gracefulPowerOffTimerExpired);
    });
}

static void powerCycleTimerStart()
{
    phosphor::logging::log<phosphor::logging::level::INFO>(
        "Power-cycle timer started");
    powerCycleTimer.expires_after(std::chrono::milliseconds(powerCycleTimeMs));
    powerCycleTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::string errMsg =
                    "Power-cycle async_wait failed: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
            }
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "Power-cycle timer canceled");
            return;
        }
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "Power-cycle timer completed");
        sendPowerControlEvent(Event::powerCycleTimerExpired);
    });
}

static void psPowerOKWatchdogTimerStart()
{
    phosphor::logging::log<phosphor::logging::level::INFO>(
        "power supply power OK watchdog timer started");
    psPowerOKWatchdogTimer.expires_after(
        std::chrono::milliseconds(psPowerOKWatchdogTimeMs));
    psPowerOKWatchdogTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::string errMsg =
                    "power supply power OK watchdog async_wait failed: " +
                    ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
            }
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "power supply power OK watchdog timer canceled");
            return;
        }
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "power supply power OK watchdog timer expired");
        sendPowerControlEvent(Event::psPowerOKWatchdogTimerExpired);
    });
}

static void warmResetCheckTimerStart()
{
    phosphor::logging::log<phosphor::logging::level::INFO>(
        "Warm reset check timer started");
    warmResetCheckTimer.expires_after(
        std::chrono::milliseconds(warmResetCheckTimeMs));
    warmResetCheckTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::string errMsg =
                    "Warm reset check async_wait failed: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
            }
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "Warm reset check timer canceled");
            return;
        }
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "Warm reset check timer completed");
        sendPowerControlEvent(Event::warmResetDetected);
    });
}

static void pohCounterTimerStart()
{
    phosphor::logging::log<phosphor::logging::level::INFO>("POH timer started");
    // Set the time-out as 1 hour, to align with POH command in ipmid
    pohCounterTimer.expires_after(std::chrono::hours(1));
    pohCounterTimer.async_wait([](const boost::system::error_code& ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::string errMsg =
                    "POH timer async_wait failed: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
            }
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "POH timer canceled");
            return;
        }

        if (getHostState(powerState) !=
            "xyz.openbmc_project.State.Host.HostState.Running")
        {
            return;
        }

        conn->async_method_call(
            [](boost::system::error_code ec,
               const std::variant<uint32_t>& pohCounterProperty) {
                if (ec)
                {
                    phosphor::logging::log<phosphor::logging::level::INFO>(
                        "error to get poh counter");
                    return;
                }
                const uint32_t* pohCounter =
                    std::get_if<uint32_t>(&pohCounterProperty);
                if (pohCounter == nullptr)
                {
                    phosphor::logging::log<phosphor::logging::level::INFO>(
                        "unable to read poh counter");
                    return;
                }

                conn->async_method_call(
                    [](boost::system::error_code ec) {
                        if (ec)
                        {
                            phosphor::logging::log<
                                phosphor::logging::level::INFO>(
                                "failed to set poh counter");
                        }
                    },
                    "xyz.openbmc_project.Settings",
                    "/xyz/openbmc_project/state/chassis0",
                    "org.freedesktop.DBus.Properties", "Set",
                    "xyz.openbmc_project.State.PowerOnHours", "POHCounter",
                    std::variant<uint32_t>(*pohCounter + 1));
            },
            "xyz.openbmc_project.Settings",
            "/xyz/openbmc_project/state/chassis0",
            "org.freedesktop.DBus.Properties", "Get",
            "xyz.openbmc_project.State.PowerOnHours", "POHCounter");

        pohCounterTimerStart();
    });
}

static void currentHostStateMonitor()
{
    if (getHostState(powerState) ==
        "xyz.openbmc_project.State.Host.HostState.Running")
    {
        pohCounterTimerStart();
        // Clear the restart cause set for the next restart
        clearRestartCause();
    }
    else
    {
        pohCounterTimer.cancel();
        // Set the restart cause set for this restart
        setRestartCause();
    }

    static auto match = sdbusplus::bus::match::match(
        *conn,
        "type='signal',member='PropertiesChanged', "
        "interface='org.freedesktop.DBus.Properties', "
        "arg0='xyz.openbmc_project.State.Host'",
        [](sdbusplus::message::message& message) {
            std::string intfName;
            std::map<std::string, std::variant<std::string>> properties;

            try
            {
                message.read(intfName, properties);
            }
            catch (std::exception& e)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "Unable to read host state");
                return;
            }
            if (properties.empty())
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "ERROR: Empty PropertiesChanged signal received");
                return;
            }

            // We only want to check for CurrentHostState
            if (properties.begin()->first != "CurrentHostState")
            {
                return;
            }
            std::string* currentHostState =
                std::get_if<std::string>(&(properties.begin()->second));
            if (currentHostState == nullptr)
            {
                std::string errMsg =
                    properties.begin()->first + " property invalid";
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
                return;
            }

            if (*currentHostState ==
                "xyz.openbmc_project.State.Host.HostState.Running")
            {
                pohCounterTimerStart();
                // Clear the restart cause set for the next restart
                clearRestartCause();
                sd_journal_send("MESSAGE=Host system DC power is on",
                                "PRIORITY=%i", LOG_INFO,
                                "REDFISH_MESSAGE_ID=%s",
                                "OpenBMC.0.1.DCPowerOn", NULL);
            }
            else
            {
                pohCounterTimer.cancel();
                // POST_COMPLETE GPIO event is not working in some platforms
                // when power state is changed to OFF. This resulted in
                // 'OperatingSystemState' to stay at 'Standby', even though
                // system is OFF. Set 'OperatingSystemState' to 'Inactive'
                // if HostState is trurned to OFF.
                osIface->set_property("OperatingSystemState",
                                      std::string("Inactive"));

                // Set the restart cause set for this restart
                setRestartCause();
                resetACBootProperty();
                sd_journal_send("MESSAGE=Host system DC power is off",
                                "PRIORITY=%i", LOG_INFO,
                                "REDFISH_MESSAGE_ID=%s",
                                "OpenBMC.0.1.DCPowerOff", NULL);
            }
        });
}

static void sioPowerGoodWatchdogTimerStart()
{
    phosphor::logging::log<phosphor::logging::level::INFO>(
        "SIO power good watchdog timer started");
    sioPowerGoodWatchdogTimer.expires_after(
        std::chrono::milliseconds(sioPowerGoodWatchdogTimeMs));
    sioPowerGoodWatchdogTimer.async_wait(
        [](const boost::system::error_code ec) {
            if (ec)
            {
                // operation_aborted is expected if timer is canceled before
                // completion.
                if (ec != boost::asio::error::operation_aborted)
                {
                    std::string errMsg =
                        "SIO power good watchdog async_wait failed: " +
                        ec.message();
                    phosphor::logging::log<phosphor::logging::level::ERR>(
                        errMsg.c_str());
                }
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "SIO power good watchdog timer canceled");
                return;
            }
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "SIO power good watchdog timer completed");
            sendPowerControlEvent(Event::sioPowerGoodWatchdogTimerExpired);
        });
}

static void powerStateOn(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            setPowerState(PowerState::off);
            // DC power is unexpectedly lost, beep
            beep(beepPowerFail);
            break;
        case Event::sioS5Assert:
            setPowerState(PowerState::transitionToOff);
            addRestartCause(RestartCause::softReset);
            break;
#if USE_PLT_RST
        case Event::pltRstAssert:
#else
        case Event::postCompleteDeAssert:
#endif
            setPowerState(PowerState::checkForWarmReset);
            addRestartCause(RestartCause::softReset);
            warmResetCheckTimerStart();
            break;
        case Event::powerButtonPressed:
            setPowerState(PowerState::gracefulTransitionToOff);
            gracefulPowerOffTimerStart();
            break;
        case Event::powerOffRequest:
            setPowerState(PowerState::transitionToOff);
            forcePowerOff();
            break;
        case Event::gracefulPowerOffRequest:
            setPowerState(PowerState::gracefulTransitionToOff);
            gracefulPowerOffTimerStart();
            gracefulPowerOff();
            break;
        case Event::powerCycleRequest:
            setPowerState(PowerState::transitionToCycleOff);
            forcePowerOff();
            break;
        case Event::gracefulPowerCycleRequest:
            setPowerState(PowerState::gracefulTransitionToCycleOff);
            gracefulPowerOffTimerStart();
            gracefulPowerOff();
            break;
        case Event::resetRequest:
            reset();
            break;
        default:
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No action taken.");
            break;
    }
}

static void powerStateWaitForPSPowerOK(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKAssert:
        {
            // Cancel any GPIO assertions held during the transition
            gpioAssertTimer.cancel();
            psPowerOKWatchdogTimer.cancel();
            if (sioEnabled == true)
            {
                sioPowerGoodWatchdogTimerStart();
                setPowerState(PowerState::waitForSIOPowerGood);
            }
            else
            {
                setPowerState(PowerState::on);
            }
            break;
        }
        case Event::psPowerOKWatchdogTimerExpired:
            setPowerState(PowerState::off);
            psPowerOKFailedLog();
            break;
        case Event::sioPowerGoodAssert:
            psPowerOKWatchdogTimer.cancel();
            setPowerState(PowerState::on);
            break;
        default:
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No action taken.");
            break;
    }
}

static void powerStateWaitForSIOPowerGood(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::sioPowerGoodAssert:
            sioPowerGoodWatchdogTimer.cancel();
            setPowerState(PowerState::on);
            break;
        case Event::sioPowerGoodWatchdogTimerExpired:
            setPowerState(PowerState::off);
            systemPowerGoodFailedLog();
            break;
        default:
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No action taken.");
            break;
    }
}

static void powerStateOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKAssert:
        {
            if (sioEnabled == true)
            {
                setPowerState(PowerState::waitForSIOPowerGood);
            }
            else
            {
                setPowerState(PowerState::on);
            }
            break;
        }
        case Event::sioS5DeAssert:
            setPowerState(PowerState::waitForPSPowerOK);
            break;
        case Event::sioPowerGoodAssert:
            setPowerState(PowerState::on);
            break;
        case Event::powerButtonPressed:
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            break;
        case Event::powerOnRequest:
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            powerOn();
            break;
        default:
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No action taken.");
            break;
    }
}

static void powerStateTransitionToOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            // Cancel any GPIO assertions held during the transition
            gpioAssertTimer.cancel();
            setPowerState(PowerState::off);
            break;
        default:
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No action taken.");
            break;
    }
}

static void powerStateGracefulTransitionToOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::off);
            break;
        case Event::gracefulPowerOffTimerExpired:
            setPowerState(PowerState::on);
            break;
        case Event::powerOffRequest:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::transitionToOff);
            forcePowerOff();
            break;
        case Event::powerCycleRequest:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::transitionToCycleOff);
            forcePowerOff();
            break;
        case Event::resetRequest:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::on);
            reset();
            break;
        default:
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No action taken.");
            break;
    }
}

static void powerStateCycleOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKAssert:
        {
            powerCycleTimer.cancel();
            if (sioEnabled == true)
            {
                setPowerState(PowerState::waitForSIOPowerGood);
            }
            else
            {
                setPowerState(PowerState::on);
            }
            break;
        }
        case Event::sioS5DeAssert:
            powerCycleTimer.cancel();
            setPowerState(PowerState::waitForPSPowerOK);
            break;
        case Event::powerButtonPressed:
            powerCycleTimer.cancel();
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            break;
        case Event::powerCycleTimerExpired:
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            powerOn();
            break;
        default:
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No action taken.");
            break;
    }
}

static void powerStateTransitionToCycleOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            // Cancel any GPIO assertions held during the transition
            gpioAssertTimer.cancel();
            setPowerState(PowerState::cycleOff);
            powerCycleTimerStart();
            break;
        default:
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No action taken.");
            break;
    }
}

static void powerStateGracefulTransitionToCycleOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::cycleOff);
            powerCycleTimerStart();
            break;
        case Event::gracefulPowerOffTimerExpired:
            setPowerState(PowerState::on);
            break;
        case Event::powerOffRequest:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::transitionToOff);
            forcePowerOff();
            break;
        case Event::powerCycleRequest:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::transitionToCycleOff);
            forcePowerOff();
            break;
        case Event::resetRequest:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::on);
            reset();
            break;
        default:
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No action taken.");
            break;
    }
}

static void powerStateCheckForWarmReset(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::sioS5Assert:
            warmResetCheckTimer.cancel();
            setPowerState(PowerState::transitionToOff);
            break;
        case Event::warmResetDetected:
            setPowerState(PowerState::on);
            break;
        case Event::psPowerOKDeAssert:
            warmResetCheckTimer.cancel();
            setPowerState(PowerState::off);
            // DC power is unexpectedly lost, beep
            beep(beepPowerFail);
            break;
        default:
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No action taken.");
            break;
    }
}

static void psPowerOKHandler()
{
    gpiod::line_event gpioLineEvent = psPowerOKLine.event_read();

    Event powerControlEvent =
        gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE
            ? Event::psPowerOKAssert
            : Event::psPowerOKDeAssert;

    sendPowerControlEvent(powerControlEvent);
    psPowerOKEvent.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::string errMsg =
                    "power supply power OK handler error: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
                return;
            }
            psPowerOKHandler();
        });
}

static void sioPowerGoodHandler()
{
    gpiod::line_event gpioLineEvent = sioPowerGoodLine.event_read();

    Event powerControlEvent =
        gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE
            ? Event::sioPowerGoodAssert
            : Event::sioPowerGoodDeAssert;

    sendPowerControlEvent(powerControlEvent);
    sioPowerGoodEvent.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::string errMsg =
                    "SIO power good handler error: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
                return;
            }
            sioPowerGoodHandler();
        });
}

static void sioOnControlHandler()
{
    gpiod::line_event gpioLineEvent = sioOnControlLine.event_read();

    bool sioOnControl =
        gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE;
    std::string logMsg =
        "SIO_ONCONTROL value changed: " + std::to_string(sioOnControl);
    phosphor::logging::log<phosphor::logging::level::INFO>(logMsg.c_str());
    sioOnControlEvent.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::string errMsg =
                    "SIO ONCONTROL handler error: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
                return;
            }
            sioOnControlHandler();
        });
}

static void sioS5Handler()
{
    gpiod::line_event gpioLineEvent = sioS5Line.event_read();

    Event powerControlEvent =
        gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE
            ? Event::sioS5Assert
            : Event::sioS5DeAssert;

    sendPowerControlEvent(powerControlEvent);
    sioS5Event.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::string errMsg = "SIO S5 handler error: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
                return;
            }
            sioS5Handler();
        });
}

static void powerButtonHandler()
{
    gpiod::line_event gpioLineEvent = powerButtonLine.event_read();

    if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
    {
        powerButtonPressLog();
        powerButtonIface->set_property("ButtonPressed", true);
        if (!powerButtonMask)
        {
            sendPowerControlEvent(Event::powerButtonPressed);
            addRestartCause(RestartCause::powerButton);
        }
        else
        {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "power button press masked");
        }
    }
    else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE)
    {
        powerButtonIface->set_property("ButtonPressed", false);
    }
    powerButtonEvent.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::string errMsg =
                    "power button handler error: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
                return;
            }
            powerButtonHandler();
        });
}

static void resetButtonHandler()
{
    gpiod::line_event gpioLineEvent = resetButtonLine.event_read();

    if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
    {
        resetButtonPressLog();
        resetButtonIface->set_property("ButtonPressed", true);
        if (!resetButtonMask)
        {
            sendPowerControlEvent(Event::resetButtonPressed);
            addRestartCause(RestartCause::resetButton);
        }
        else
        {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "reset button press masked");
        }
    }
    else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE)
    {
        resetButtonIface->set_property("ButtonPressed", false);
    }
    resetButtonEvent.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::string errMsg =
                    "reset button handler error: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
                return;
            }
            resetButtonHandler();
        });
}

#ifdef CHASSIS_SYSTEM_RESET
static constexpr auto systemdBusname = "org.freedesktop.systemd1";
static constexpr auto systemdPath = "/org/freedesktop/systemd1";
static constexpr auto systemdInterface = "org.freedesktop.systemd1.Manager";
static constexpr auto systemTargetName = "chassis-system-reset.target";

void systemReset()
{
    conn->async_method_call(
        [](boost::system::error_code ec) {
            if (ec)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "Failed to call chassis system reset",
                    phosphor::logging::entry("ERR=%s", ec.message().c_str()));
            }
        },
        systemdBusname, systemdPath, systemdInterface, "StartUnit",
        systemTargetName, "replace");
}
#endif

static void nmiSetEnableProperty(bool value)
{
    conn->async_method_call(
        [](boost::system::error_code ec) {
            if (ec)
            {
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "failed to set NMI source");
            }
        },
        "xyz.openbmc_project.Settings",
        "/xyz/openbmc_project/Chassis/Control/NMISource",
        "org.freedesktop.DBus.Properties", "Set",
        "xyz.openbmc_project.Chassis.Control.NMISource", "Enabled",
        std::variant<bool>{value});
}

static void nmiReset(void)
{
    static constexpr const uint8_t value = 1;
    const static constexpr int nmiOutPulseTimeMs = 200;

    phosphor::logging::log<phosphor::logging::level::INFO>("NMI out action");
    nmiOutLine.set_value(value);
    std::string logMsg = nmiOutName + " set to " + std::to_string(value);
    phosphor::logging::log<phosphor::logging::level::INFO>(logMsg.c_str());
    gpioAssertTimer.expires_after(std::chrono::milliseconds(nmiOutPulseTimeMs));
    gpioAssertTimer.async_wait([](const boost::system::error_code ec) {
        // restore the NMI_OUT GPIO line back to the opposite value
        nmiOutLine.set_value(!value);
        std::string logMsg = nmiOutName + " released";
        phosphor::logging::log<phosphor::logging::level::INFO>(logMsg.c_str());
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::string errMsg =
                    nmiOutName + " async_wait failed: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
            }
        }
    });
    // log to redfish
    nmiDiagIntLog();
    phosphor::logging::log<phosphor::logging::level::INFO>(
        "NMI out action completed");
    // reset Enable Property
    nmiSetEnableProperty(false);
}

static void nmiSourcePropertyMonitor(void)
{
    phosphor::logging::log<phosphor::logging::level::INFO>(
        "NMI Source Property Monitor");

    static std::unique_ptr<sdbusplus::bus::match::match> nmiSourceMatch =
        std::make_unique<sdbusplus::bus::match::match>(
            *conn,
            "type='signal',interface='org.freedesktop.DBus.Properties',"
            "member='PropertiesChanged',arg0namespace='xyz.openbmc_project."
            "Chassis.Control."
            "NMISource'",
            [](sdbusplus::message::message& msg) {
                std::string interfaceName;
                boost::container::flat_map<std::string,
                                           std::variant<bool, std::string>>
                    propertiesChanged;
                std::string state;
                bool value = true;
                try
                {
                    msg.read(interfaceName, propertiesChanged);
                    if (propertiesChanged.begin()->first == "Enabled")
                    {
                        value =
                            std::get<bool>(propertiesChanged.begin()->second);
                        std::string logMsg =
                            " NMI Enabled propertiesChanged value: " +
                            std::to_string(value);
                        phosphor::logging::log<phosphor::logging::level::INFO>(
                            logMsg.c_str());
                        nmiEnabled = value;
                        if (nmiEnabled)
                        {
                            nmiReset();
                        }
                    }
                }
                catch (std::exception& e)
                {
                    phosphor::logging::log<phosphor::logging::level::ERR>(
                        "Unable to read NMI source");
                    return;
                }
            });
}

static void setNmiSource()
{
    conn->async_method_call(
        [](boost::system::error_code ec) {
            if (ec)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "failed to set NMI source");
            }
        },
        "xyz.openbmc_project.Settings",
        "/xyz/openbmc_project/Chassis/Control/NMISource",
        "org.freedesktop.DBus.Properties", "Set",
        "xyz.openbmc_project.Chassis.Control.NMISource", "BMCSource",
        std::variant<std::string>{"xyz.openbmc_project.Chassis.Control."
                                  "NMISource.BMCSourceSignal.FpBtn"});
    // set Enable Property
    nmiSetEnableProperty(true);
}

static void nmiButtonHandler()
{
    gpiod::line_event gpioLineEvent = nmiButtonLine.event_read();

    if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
    {
        nmiButtonPressLog();
        nmiButtonIface->set_property("ButtonPressed", true);
        if (nmiButtonMasked)
        {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "NMI button press masked");
        }
        else
        {
            setNmiSource();
        }
    }
    else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE)
    {
        nmiButtonIface->set_property("ButtonPressed", false);
    }
    nmiButtonEvent.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::string errMsg =
                    "NMI button handler error: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
                return;
            }
            nmiButtonHandler();
        });
}

static void idButtonHandler()
{
    gpiod::line_event gpioLineEvent = idButtonLine.event_read();

    if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
    {
        idButtonIface->set_property("ButtonPressed", true);
    }
    else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE)
    {
        idButtonIface->set_property("ButtonPressed", false);
    }
    idButtonEvent.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code& ec) {
            if (ec)
            {
                std::string errMsg = "ID button handler error: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
                return;
            }
            idButtonHandler();
        });
}

static void pltRstHandler(bool pltRst)
{
    if (pltRst)
    {
        sendPowerControlEvent(Event::pltRstDeAssert);
    }
    else
    {
        sendPowerControlEvent(Event::pltRstAssert);
    }
}

static void hostMiscHandler(sdbusplus::message::message& msg)
{
    std::string interfaceName;
    boost::container::flat_map<std::string, std::variant<bool>>
        propertiesChanged;
    bool pltRst;
    try
    {
        msg.read(interfaceName, propertiesChanged);
    }
    catch (std::exception& e)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Unable to read Host Misc status");
        return;
    }
    if (propertiesChanged.empty())
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "ERROR: Empty Host.Misc PropertiesChanged signal received");
        return;
    }

    for (auto& [property, value] : propertiesChanged)
    {
        if (property == "ESpiPlatformReset")
        {
            bool* pltRst = std::get_if<bool>(&value);
            if (pltRst == nullptr)
            {
                std::string errMsg = property + " property invalid";
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
                return;
            }
            pltRstHandler(*pltRst);
        }
    }
}

static void postCompleteHandler()
{
    gpiod::line_event gpioLineEvent = postCompleteLine.event_read();

    bool postComplete =
        gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE;
    if (postComplete)
    {
        sendPowerControlEvent(Event::postCompleteAssert);
        osIface->set_property("OperatingSystemState", std::string("Standby"));
    }
    else
    {
        sendPowerControlEvent(Event::postCompleteDeAssert);
        osIface->set_property("OperatingSystemState", std::string("Inactive"));
    }
    postCompleteEvent.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::string errMsg =
                    "POST complete handler error: " + ec.message();
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    errMsg.c_str());
                return;
            }
            postCompleteHandler();
        });
}

static int loadConfigValues()
{
    const std::string configFilePath =
        "/usr/share/x86-power-control/power-config-host" + power_control::node +
        ".json";
    std::ifstream configFile(configFilePath.c_str());
    if (!configFile.is_open())
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "loadConfigValues : Cannot open config path");
        return -1;
    }
    auto data = nlohmann::json::parse(configFile, nullptr);

    if (data.is_discarded())
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Power config readings JSON parser failure");
        return -1;
    }
    auto gpios = data["gpio_configs"];
    auto timers = data["timing_configs"];

    if (gpios.contains("IdButton"))
    {
        idButtonName = gpios["IdButton"];
    }

    if (gpios.contains("NMIButton"))
    {
        nmiButtonName = gpios["NMIButton"];
    }

    if (gpios.contains("NMIOut"))
    {
        nmiOutName = gpios["NMIOut"];
    }

    if (gpios.contains("PostComplete"))
    {
        postCompleteName = gpios["PostComplete"];
    }

    if (gpios.contains("PwrButton"))
    {
        powerButtonName = gpios["PwrButton"];
    }

    if (gpios.contains("PwrOK"))
    {
        powerOkName = gpios["PwrOK"];
    }

    if (gpios.contains("PwrOut"))
    {
        powerOutName = gpios["PwrOut"];
    }

    if (gpios.contains("RstButton"))
    {
        resetButtonName = gpios["RstButton"];
    }

    if (gpios.contains("RstOut"))
    {
        resetOutName = gpios["RstOut"];
    }

    if (gpios.contains("SIOOnCtl"))
    {
        sioOnControlName = gpios["SIOOnCtl"];
    }

    if (gpios.contains("SIOPwrGd"))
    {
        sioPwrGoodName = gpios["SIOPwrGd"];
    }

    if (gpios.contains("SIOS5"))
    {
        sioS5Name = gpios["SIOS5"];
    }

    if (timers.contains("PowerPulseMs"))
    {
        powerPulseTimeMs = timers["PowerPulseMs"];
    }

    if (timers.contains("ForceOffPulseMs"))
    {
        forceOffPulseTimeMs = timers["ForceOffPulseMs"];
    }

    if (timers.contains("ResetPulseMs"))
    {
        resetPulseTimeMs = timers["ResetPulseMs"];
    }

    if (timers.contains("PowerCycleMs"))
    {
        powerCycleTimeMs = timers["PowerCycleMs"];
    }

    if (timers.contains("SioPowerGoodWatchdogMs"))
    {
        sioPowerGoodWatchdogTimeMs = timers["SioPowerGoodWatchdogMs"];
    }

    if (timers.contains("PsPowerOKWatchdogMs"))
    {
        psPowerOKWatchdogTimeMs = timers["PsPowerOKWatchdogMs"];
    }

    if (timers.contains("GracefulPowerOffS"))
    {
        gracefulPowerOffTimeS = timers["GracefulPowerOffS"];
    }

    if (timers.contains("WarmResetCheckMs"))
    {
        warmResetCheckTimeMs = timers["WarmResetCheckMs"];
    }

    if (timers.contains("PowerOffSaveMs"))
    {
        powerOffSaveTimeMs = timers["PowerOffSaveMs"];
    }
    return 0;
}

} // namespace power_control

int main(int argc, char* argv[])
{
    using namespace power_control;
    phosphor::logging::log<phosphor::logging::level::INFO>(
        "Start Chassis power control service...");
    conn = std::make_shared<sdbusplus::asio::connection>(io);

    // Load GPIO's through json config file
    if (loadConfigValues() == -1)
    {
        std::string errMsg = "Host" + node + ": " + "Error in Parsing...";
        phosphor::logging::log<phosphor::logging::level::ERR>(errMsg.c_str());
    }

    // Request all the dbus names
    conn->request_name("xyz.openbmc_project.State.Host");
    conn->request_name("xyz.openbmc_project.State.Chassis");
    conn->request_name("xyz.openbmc_project.State.OperatingSystem");
    conn->request_name("xyz.openbmc_project.Chassis.Buttons");
    conn->request_name("xyz.openbmc_project.Control.Host.NMI");
    conn->request_name("xyz.openbmc_project.Control.Host.RestartCause");

    if (sioPwrGoodName.empty() || sioOnControlName.empty() || sioS5Name.empty())
    {
        sioEnabled = false;
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "SIO control GPIOs not defined, disable SIO support.");
    }

    // Request PS_PWROK GPIO events
    if (!powerOkName.empty())
    {
        if (!requestGPIOEvents(powerOkName, psPowerOKHandler, psPowerOKLine,
                               psPowerOKEvent))
        {
            return -1;
        }
    }
    else
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "PowerOk name should be configured from json config file");
        return -1;
    }

    if (sioEnabled == true)
    {
        // Request SIO_POWER_GOOD GPIO events
        if (!requestGPIOEvents(sioPwrGoodName, sioPowerGoodHandler,
                               sioPowerGoodLine, sioPowerGoodEvent))
        {
            return -1;
        }

        // Request SIO_ONCONTROL GPIO events
        if (!requestGPIOEvents(sioOnControlName, sioOnControlHandler,
                               sioOnControlLine, sioOnControlEvent))
        {
            return -1;
        }

        // Request SIO_S5 GPIO events
        if (!requestGPIOEvents(sioS5Name, sioS5Handler, sioS5Line, sioS5Event))
        {
            return -1;
        }
    }

    // Request POWER_BUTTON GPIO events
    if (!powerButtonName.empty())
    {
        if (!requestGPIOEvents(powerButtonName, powerButtonHandler,
                               powerButtonLine, powerButtonEvent))
        {
            return -1;
        }
    }
    else
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "powerButton name should be configured from json config file");
        return -1;
    }

    // Request RESET_BUTTON GPIO events
    if (!resetButtonName.empty())
    {
        if (!requestGPIOEvents(resetButtonName, resetButtonHandler,
                               resetButtonLine, resetButtonEvent))
        {
            return -1;
        }
    }
    else
    {
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "ResetButton not defined...");
    }

    // Request NMI_BUTTON GPIO events
    if (!nmiButtonName.empty())
    {
        requestGPIOEvents(nmiButtonName, nmiButtonHandler, nmiButtonLine,
                          nmiButtonEvent);
    }

    // Request ID_BUTTON GPIO events
    if (!idButtonName.empty())
    {
        requestGPIOEvents(idButtonName, idButtonHandler, idButtonLine,
                          idButtonEvent);
    }

#ifdef USE_PLT_RST
    sdbusplus::bus::match::match pltRstMatch(
        *conn,
        "type='signal',interface='org.freedesktop.DBus.Properties',member='"
        "PropertiesChanged',arg0='xyz.openbmc_project.State.Host.Misc'",
        hostMiscHandler);
#endif

    // Request POST_COMPLETE GPIO events
    if (!postCompleteName.empty())
    {
        if (!requestGPIOEvents(postCompleteName, postCompleteHandler,
                               postCompleteLine, postCompleteEvent))
        {
            return -1;
        }
    }
    else
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "postComplete name should be configured from json config file");
        return -1;
    }

    // initialize NMI_OUT GPIO.
    setGPIOOutput(nmiOutName, 0, nmiOutLine);

    // Initialize POWER_OUT and RESET_OUT GPIO.
    gpiod::line line;
    if (!setGPIOOutput(powerOutName, 1, line))
    {
        return -1;
    }

    if (!setGPIOOutput(resetOutName, 1, line))
    {
        return -1;
    }

    // Release line
    line.reset();

    // Initialize the power state
    powerState = PowerState::off;
    // Check power good
    if (psPowerOKLine.get_value() > 0)
    {
        powerState = PowerState::on;
    }

    // Initialize the power state storage
    if (initializePowerStateStorage() < 0)
    {
        return -1;
    }

    // Check if we need to start the Power Restore policy
    powerRestorePolicyCheck();

    if (nmiOutLine)
        nmiSourcePropertyMonitor();

    phosphor::logging::log<phosphor::logging::level::INFO>(
        "Initializing power state. ");
    logStateTransition(powerState);

    // Power Control Service
    sdbusplus::asio::object_server hostServer =
        sdbusplus::asio::object_server(conn);

    // Power Control Interface
    hostIface = hostServer.add_interface("/xyz/openbmc_project/state/host0",
                                         "xyz.openbmc_project.State.Host");

    hostIface->register_property(
        "RequestedHostTransition",
        std::string("xyz.openbmc_project.State.Host.Transition.Off"),
        [](const std::string& requested, std::string& resp) {
            if (requested == "xyz.openbmc_project.State.Host.Transition.Off")
            {
                sendPowerControlEvent(Event::gracefulPowerOffRequest);
                addRestartCause(RestartCause::command);
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Host.Transition.On")
            {
                sendPowerControlEvent(Event::powerOnRequest);
                addRestartCause(RestartCause::command);
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Host.Transition.Reboot")
            {
                sendPowerControlEvent(Event::powerCycleRequest);
                addRestartCause(RestartCause::command);
            }
            else if (requested == "xyz.openbmc_project.State.Host.Transition."
                                  "GracefulWarmReboot")
            {
                sendPowerControlEvent(Event::gracefulPowerCycleRequest);
                addRestartCause(RestartCause::command);
            }
            else if (requested == "xyz.openbmc_project.State.Host.Transition."
                                  "ForceWarmReboot")
            {
                sendPowerControlEvent(Event::resetRequest);
                addRestartCause(RestartCause::command);
            }
            else
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "Unrecognized host state transition request.");
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
            resp = requested;
            return 1;
        });
    hostIface->register_property("CurrentHostState",
                                 std::string(getHostState(powerState)));

    hostIface->initialize();

    // Chassis Control Service
    sdbusplus::asio::object_server chassisServer =
        sdbusplus::asio::object_server(conn);

    // Chassis Control Interface
    chassisIface =
        chassisServer.add_interface("/xyz/openbmc_project/state/chassis0",
                                    "xyz.openbmc_project.State.Chassis");

    chassisIface->register_property(
        "RequestedPowerTransition",
        std::string("xyz.openbmc_project.State.Chassis.Transition.Off"),
        [](const std::string& requested, std::string& resp) {
            if (requested == "xyz.openbmc_project.State.Chassis.Transition.Off")
            {
                sendPowerControlEvent(Event::powerOffRequest);
                addRestartCause(RestartCause::command);
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Chassis.Transition.On")
            {
                sendPowerControlEvent(Event::powerOnRequest);
                addRestartCause(RestartCause::command);
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Chassis.Transition.PowerCycle")
            {
                sendPowerControlEvent(Event::powerCycleRequest);
                addRestartCause(RestartCause::command);
            }
            else
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "Unrecognized chassis state transition request.");
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
            resp = requested;
            return 1;
        });
    chassisIface->register_property("CurrentPowerState",
                                    std::string(getChassisState(powerState)));
    chassisIface->register_property("LastStateChangeTime", getCurrentTimeMs());

    chassisIface->initialize();

#ifdef CHASSIS_SYSTEM_RESET
    // Chassis System Service
    sdbusplus::asio::object_server chassisSysServer =
        sdbusplus::asio::object_server(conn);

    // Chassis System Interface
    chassisSysIface = chassisSysServer.add_interface(
        "/xyz/openbmc_project/state/chassis_system0",
        "xyz.openbmc_project.State.Chassis");

    chassisSysIface->register_property(
        "RequestedPowerTransition",
        std::string("xyz.openbmc_project.State.Chassis.Transition.On"),
        [](const std::string& requested, std::string& resp) {
            if (requested ==
                "xyz.openbmc_project.State.Chassis.Transition.PowerCycle")
            {
                systemReset();
                addRestartCause(RestartCause::command);
            }
            else
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "Unrecognized chassis system state transition request.");
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
            resp = requested;
            return 1;
        });
    chassisSysIface->register_property(
        "CurrentPowerState", std::string(getChassisState(powerState)));
    chassisSysIface->register_property("LastStateChangeTime",
                                       getCurrentTimeMs());

    chassisSysIface->initialize();
#endif

    // Buttons Service
    sdbusplus::asio::object_server buttonsServer =
        sdbusplus::asio::object_server(conn);

    // Power Button Interface
    powerButtonIface = buttonsServer.add_interface(
        "/xyz/openbmc_project/chassis/buttons/power",
        "xyz.openbmc_project.Chassis.Buttons");

    powerButtonIface->register_property(
        "ButtonMasked", false, [](const bool requested, bool& current) {
            if (requested)
            {
                if (powerButtonMask)
                {
                    return 1;
                }
                if (!setGPIOOutput(powerOutName, 1, powerButtonMask))
                {
                    throw std::runtime_error("Failed to request GPIO");
                    return 0;
                }
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "Power Button Masked.");
            }
            else
            {
                if (!powerButtonMask)
                {
                    return 1;
                }
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "Power Button Un-masked");
                powerButtonMask.reset();
            }
            // Update the mask setting
            current = requested;
            return 1;
        });

    // Check power button state
    bool powerButtonPressed = powerButtonLine.get_value() == 0;
    powerButtonIface->register_property("ButtonPressed", powerButtonPressed);

    powerButtonIface->initialize();

    // Reset Button Interface
    if (!resetButtonName.empty())
    {
        resetButtonIface = buttonsServer.add_interface(
            "/xyz/openbmc_project/chassis/buttons/reset",
            "xyz.openbmc_project.Chassis.Buttons");

        resetButtonIface->register_property(
            "ButtonMasked", false, [](const bool requested, bool& current) {
                if (requested)
                {
                    if (resetButtonMask)
                    {
                        return 1;
                    }
                    if (!setGPIOOutput(resetOutName, 1, resetButtonMask))
                    {
                        throw std::runtime_error("Failed to request GPIO");
                        return 0;
                    }
                    phosphor::logging::log<phosphor::logging::level::INFO>(
                        "Reset Button Masked.");
                }
                else
                {
                    if (!resetButtonMask)
                    {
                        return 1;
                    }
                    phosphor::logging::log<phosphor::logging::level::INFO>(
                        "Reset Button Un-masked");
                    resetButtonMask.reset();
                }
                // Update the mask setting
                current = requested;
                return 1;
            });

        // Check reset button state
        bool resetButtonPressed = resetButtonLine.get_value() == 0;
        resetButtonIface->register_property("ButtonPressed",
                                            resetButtonPressed);

        resetButtonIface->initialize();
    }

    if (nmiButtonLine)
    {
        // NMI Button Interface
        nmiButtonIface = buttonsServer.add_interface(
            "/xyz/openbmc_project/chassis/buttons/nmi",
            "xyz.openbmc_project.Chassis.Buttons");

        nmiButtonIface->register_property(
            "ButtonMasked", false, [](const bool requested, bool& current) {
                if (nmiButtonMasked == requested)
                {
                    // NMI button mask is already set as requested, so no change
                    return 1;
                }
                if (requested)
                {
                    phosphor::logging::log<phosphor::logging::level::INFO>(
                        "NMI Button Masked.");
                    nmiButtonMasked = true;
                }
                else
                {
                    phosphor::logging::log<phosphor::logging::level::INFO>(
                        "NMI Button Un-masked.");
                    nmiButtonMasked = false;
                }
                // Update the mask setting
                current = nmiButtonMasked;
                return 1;
            });

        // Check NMI button state
        bool nmiButtonPressed = nmiButtonLine.get_value() == 0;
        nmiButtonIface->register_property("ButtonPressed", nmiButtonPressed);

        nmiButtonIface->initialize();
    }

    if (nmiOutLine)
    {
        // NMI out Service
        sdbusplus::asio::object_server nmiOutServer =
            sdbusplus::asio::object_server(conn);

        // NMI out Interface
        nmiOutIface =
            nmiOutServer.add_interface("/xyz/openbmc_project/control/host0/nmi",
                                       "xyz.openbmc_project.Control.Host.NMI");
        nmiOutIface->register_method("NMI", nmiReset);
        nmiOutIface->initialize();
    }

    if (idButtonLine)
    {
        // ID Button Interface
        idButtonIface = buttonsServer.add_interface(
            "/xyz/openbmc_project/chassis/buttons/id",
            "xyz.openbmc_project.Chassis.Buttons");

        // Check ID button state
        bool idButtonPressed = idButtonLine.get_value() == 0;
        idButtonIface->register_property("ButtonPressed", idButtonPressed);

        idButtonIface->initialize();
    }

    // OS State Service
    sdbusplus::asio::object_server osServer =
        sdbusplus::asio::object_server(conn);

    // OS State Interface
    osIface = osServer.add_interface(
        "/xyz/openbmc_project/state/os",
        "xyz.openbmc_project.State.OperatingSystem.Status");

    // Get the initial OS state based on POST complete
    //      0: Asserted, OS state is "Standby" (ready to boot)
    //      1: De-Asserted, OS state is "Inactive"
    std::string osState =
        postCompleteLine.get_value() > 0 ? "Inactive" : "Standby";

    osIface->register_property("OperatingSystemState", std::string(osState));

    osIface->initialize();

    // Restart Cause Service
    sdbusplus::asio::object_server restartCauseServer =
        sdbusplus::asio::object_server(conn);

    // Restart Cause Interface
    restartCauseIface = restartCauseServer.add_interface(
        "/xyz/openbmc_project/control/host0/restart_cause",
        "xyz.openbmc_project.Control.Host.RestartCause");

    restartCauseIface->register_property(
        "RestartCause",
        std::string("xyz.openbmc_project.State.Host.RestartCause.Unknown"));

    restartCauseIface->register_property(
        "RequestedRestartCause",
        std::string("xyz.openbmc_project.State.Host.RestartCause.Unknown"),
        [](const std::string& requested, std::string& resp) {
            if (requested ==
                "xyz.openbmc_project.State.Host.RestartCause.WatchdogTimer")
            {
                addRestartCause(RestartCause::watchdog);
            }
            else
            {
                throw std::invalid_argument(
                    "Unrecognized RestartCause Request");
                return 0;
            }

            std::string logMsg = "RestartCause requested: " + requested;
            phosphor::logging::log<phosphor::logging::level::INFO>(
                logMsg.c_str());
            resp = requested;
            return 1;
        });

    restartCauseIface->initialize();

    currentHostStateMonitor();

    io.run();

    return 0;
}
