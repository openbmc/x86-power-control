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
#include "power_control.hpp"

bool PowerControl::forcePowerOff()
{
    return true;
}

int32_t PowerControl::triggerReset()
{
    int ret = 0;
    int count = 0;
    char buf = '0';

    phosphor::logging::log<phosphor::logging::level::DEBUG>("triggerReset");

    ret = ::lseek(reset_out_fd, 0, SEEK_SET);
    if (ret < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("lseek error!");
        throw sdbusplus::xyz::openbmc_project::Chassis::Common::Error::
            IOError();
    }

    buf = '0';

    ret = ::write(reset_out_fd, &buf, sizeof(buf));
    if (ret < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("write error!");
        throw sdbusplus::xyz::openbmc_project::Chassis::Common::Error::
            IOError();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(RESET_PULSE_TIME_MS));

    buf = '1';
    ret = ::write(reset_out_fd, &buf, sizeof(buf));
    if (ret < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("write error!");
        throw sdbusplus::xyz::openbmc_project::Chassis::Common::Error::
            IOError();
    }
    return 0;
}

int32_t PowerControl::setPowerState(int newState)
{
    int ret = 0;
    int count = 0;
    char buf = '0';

    if (newState < 0 || newState >= powerStateMax)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "error! invalid parameter!");
        return -1;
    }

    phosphor::logging::log<phosphor::logging::level::DEBUG>(
        "setPowerState", phosphor::logging::entry("NEWSTATE=%d", newState));

    if (powerStateReset == newState)
    {
        phosphor::logging::log<phosphor::logging::level::DEBUG>(
            "setPowerState system reset");
        triggerReset();
        return 0;
    }

    if (state() == newState)
    {
        phosphor::logging::log<phosphor::logging::level::WARNING>(
            "Same powerstate",
            phosphor::logging::entry("NEWSTATE=%d", newState));
        return 0;
    }

    state(newState);

    ret = ::lseek(power_up_fd, 0, SEEK_SET);
    if (ret < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("lseek error!");
        throw sdbusplus::xyz::openbmc_project::Chassis::Common::Error::
            IOError();
    }

    buf = '0';

    ret = ::write(power_up_fd, &buf, sizeof(buf));
    if (ret < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("write error!");
        throw sdbusplus::xyz::openbmc_project::Chassis::Common::Error::
            IOError();
    }

    phosphor::logging::log<phosphor::logging::level::DEBUG>(
        "setPowerState power on");
    std::this_thread::sleep_for(std::chrono::milliseconds(POWER_PULSE_TIME_MS));

    buf = '1';
    ret = ::write(power_up_fd, &buf, sizeof(buf));
    if (ret < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("write error!");
        throw sdbusplus::xyz::openbmc_project::Chassis::Common::Error::
            IOError();
    }

    if (0 == newState)
    {
        /*
         * For power off, currently there is a known issue, the "long-press"
         * power button cannot power off the host, a workaround is perform force
         * power off after waitting for a while
         */
        std::this_thread::sleep_for(
            std::chrono::milliseconds(POWER_PULSE_TIME_MS));
        if (1 == pGood())
        { // still on, force off!
            phosphor::logging::log<phosphor::logging::level::DEBUG>(
                "Perform force power off");
            count = 0;
            do
            {
                if (count++ > 5)
                {
                    phosphor::logging::log<phosphor::logging::level::ERR>(
                        "forcePowerOff error!");
                    throw sdbusplus::xyz::openbmc_project::Chassis::Common::
                        Error::IOError();
                }
                ret = forcePowerOff();
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(POLLING_INTERVAL_MS));
            } while (ret != 0);
        }
    }

    return 0;

}

