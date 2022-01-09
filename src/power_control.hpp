/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#pragma once

#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <nlohmann/json.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <filesystem>
#include <string_view>

namespace power_control
{

using dbusPropertiesList =
    boost::container::flat_map<std::string,
                               std::variant<std::string, uint16_t>>;
/**
 * @brief The class contains functions to invoke power restore policy.
 *
 * This class only exists to unite all PowerRestore-related code. It supposed
 * to run only once on application startup.
 */
class PowerRestoreController
{
  public:
    PowerRestoreController(boost::asio::io_service& io) :
        policyInvoked(false), powerRestoreDelay(-1), powerRestoreTimer(io),
        timerState(TimerState::NotStarted)
    {}
    /**
     * @brief Power Restore entry point.
     *
     * Call this to start Power Restore algorithm.
     */
    void run();
    /**
     * @brief Initialize configuration parameters.
     *
     * Parse list of properties, received from dbus, to set Power Restore
     * algorithm configuration.
     * @param props - map of property names and values
     */
    void setProperties(const dbusPropertiesList& props);

  private:
    enum class TimerState
    {
        NotStarted,
        Running,
        Finished
    };

    bool policyInvoked;
    std::string powerRestorePolicy;
    int powerRestoreDelay;
    std::list<sdbusplus::bus::match::match> matches;
    boost::asio::steady_timer powerRestoreTimer;
    TimerState timerState;
#ifdef USE_ACBOOT
    std::string acBoot;
#endif // USE_ACBOOT

    /**
     * @brief Check if all required algorithms parameters are set
     *
     * Call this after set any of Power Restore algorithm parameters. Once all
     * parameters are set this will run invoke() function.
     */
    void invokeIfReady();
    /**
     * @brief Actually perform power restore actions.
     *
     * Take Power Restore actions according to Policy and other parameters.
     */
    void invoke();
    /**
     * @brief Check if power was dropped.
     *
     * Read last saved power state to determine if host power was enabled before
     * last BMC reboot.
     */
    bool wasPowerDropped();
};

class PersistentState
{
  public:
    enum class Params
    {
        PowerState,
        LastNMISource,
    };

    PersistentState();
    ~PersistentState();
    const std::string get(Params parameter);
    void set(Params parameter, const std::string& value);

  private:
    nlohmann::json stateData;
    const std::filesystem::path powerControlDir = "/var/lib/power-control";
    const std::string_view stateFile = "state.json";
    const int indentationSize = 2;

    const std::string getName(const Params parameter);
    const std::string getDefault(const Params parameter);
    void saveState();
};

} // namespace power_control
