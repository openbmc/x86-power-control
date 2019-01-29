/*
// Copyright (c) 2018 Intel Corporation
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

#pragma once
#include "gpio.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <phosphor-logging/elog-errors.hpp>
#include <xyz/openbmc_project/Chassis/Common/error.hpp>
#include <xyz/openbmc_project/Chassis/Control/Power/server.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

static constexpr size_t POLLING_INTERVAL_MS = 500;
const static constexpr char* PGOOD_PIN = "PGOOD";
const static constexpr char* POWER_UP_PIN = "POWER_UP_PIN";

const static constexpr size_t POWER_PULSE_TIME_MS = 200;
const static constexpr size_t RESET_PULSE_TIME_MS = 500;
const static constexpr char *PowerControlPath =
    "/xyz/openbmc_project/Chassis/Control/Power0";
const static constexpr char *PowerControlIntf =
    "xyz.openbmc_project.Chassis.Control.Power";
const static constexpr uint8_t powerStateOff = 0;
const static constexpr uint8_t powerStateOn = 1;
const static constexpr uint8_t powerStateReset = 2;
const static constexpr uint8_t powerStateMax = 3;

struct EventDeleter
{
    void operator()(sd_event *event) const
    {
        event = sd_event_unref(event);
    }
};

using EventPtr = std::unique_ptr<sd_event, EventDeleter>;


using pwr_control =
    sdbusplus::xyz::openbmc_project::Chassis::Control::server::Power;

struct PowerControl : sdbusplus::server::object_t<pwr_control>
{
    PowerControl(sdbusplus::bus::bus& bus, const char* path,
                 EventPtr &event,
                 //phosphor::watchdog::EventPtr event,
                 sd_event_io_handler_t handler = PowerControl::EventHandler) :
        sdbusplus::server::object_t<pwr_control>(bus, path),
        bus(bus), callbackHandler(handler),
        propertiesChangedSignal(
            bus,
            sdbusplus::bus::match::rules::type::signal() +
                sdbusplus::bus::match::rules::member("PropertiesChanged") +
                sdbusplus::bus::match::rules::path(
                    PowerControlPath) +
                sdbusplus::bus::match::rules::interface(PowerControlIntf),
            [this](sdbusplus::message::message &msg) {
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "PowerControl propertiesChangedSignal callback function is "
                    "called...");
                
                std::map<std::string, sdbusplus::message::variant<int, bool, std::string>> msgData;
                msg.read(objectName, msgData);
                // Check if it was the Value property that changed.
                auto valPropMap = msgData.find("State");
                {
                    if (valPropMap != msgData.end())
                    {
                        this->setPowerState(valPropMap->second);
                    }
                }
        })
    {
        int ret = -1;
        char buf = '0';

        // config gpio
        ret = configGpio(PGOOD_PIN, &pgood_fd, bus);
        if (ret < 0)
        {
            throw std::runtime_error("failed to config PGOOD_PIN");
        }

        ret = configGpio(POWER_UP_PIN, &power_up_fd, bus);
        if (ret < 0)
        {
            closeGpio(pgood_fd);
            throw std::runtime_error("failed to config POWER_UP_PIN");
        }
/*
        ret = sd_event_add_io(event.get(), nullptr, pgood_fd, EPOLLPRI,
                              callbackHandler, this);
        if (ret < 0)
        {
            closeGpio(pgood_fd);
            closeGpio(power_up_fd);
            throw std::runtime_error("failed to add to event loop");
        }

        timer.start(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::milliseconds(POLLING_INTERVAL_MS)));
        timer.setEnabled<std::true_type>();
        phosphor::logging::log<phosphor::logging::level::DEBUG>("Enable timer");
*/
    }

    ~PowerControl()
    {
        closeGpio(pgood_fd);
        closeGpio(power_up_fd);
    }

    static int EventHandler(sd_event_source* es, int fd, uint32_t revents,
                            void* userdata)
    {
        // For the first event, only set the initial status,  do not emit signal
        // since is it not triggered by the real gpio change
        static bool first_event = true;
        int n = -1;
        char buf = '0';

        if (!userdata)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "userdata null!");
            return -1;
        }

        PowerControl* powercontrol = static_cast<PowerControl*>(userdata);

        if (!powercontrol)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "null pointer!");
            return -1;
        }

        n = ::lseek(fd, 0, SEEK_SET);
        if (n < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "lseek error!");
            return n;
        }

        n = ::read(fd, &buf, sizeof(buf));
        if (n < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "read error!");
            return n;
        }

        if (buf == '0')
        {
            powercontrol->state(0);
            powercontrol->pGood(0);

            if (first_event)
            {
                first_event = false;
            }
            else
            {
                //powercontrol->powerLost();
            }
        }
        else
        {
            powercontrol->state(1);
            powercontrol->pGood(1);
            if (first_event)
            {
                first_event = false;
            }
            else
            {
//                powercontrol->powerGood();
            }
        }

        return 0;
    }

    bool forcePowerOff() override;

  private:
    int reset_out_fd;
    int power_up_fd;
    int pgood_fd;
    sdbusplus::bus::bus& bus;
    sd_event_io_handler_t callbackHandler;
    int32_t setPowerState(int32_t newState) ;
    int32_t triggerReset();
};
