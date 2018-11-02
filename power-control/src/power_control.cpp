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

int32_t PowerControl::setPowerState(int32_t newState)
{
    int ret = 0;
    int count = 0;
    char buf = '0';

    phosphor::logging::log<phosphor::logging::level::DEBUG>(
        "setPowerState", phosphor::logging::entry("NEWSTATE=%d", newState));

    if (state() == newState)
    {
        phosphor::logging::log<phosphor::logging::level::WARNING>(
            "Same powerstate",
            phosphor::logging::entry("NEWSTATE=%d", newState));
        return 0;
    }

    ret = ::lseek(power_up_fd, 0, SEEK_SET);
    if (ret < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("lseek error!");
        throw sdbusplus::xyz::openbmc_project::Chassis::Common::Error::
            IOError();
    }

    /*
    This power control just handle out pin "POWER_UP_PIN", change it
    to low "0" form high "1" and set it back to high after over 200ms,
    which will notify host (PCH) to switch power. Host to determine it
    is power on or power off operation based on current power status.
    For BMC (power control), just need to notify host (PCH) to switch
    power, don't need to judge it should power on or off.
    */
    buf = '0';
    ret = ::write(power_up_fd, &buf, sizeof(buf));
    if (ret < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "write error for setting 0 !");
        throw sdbusplus::xyz::openbmc_project::Chassis::Common::Error::
            IOError();
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(POWER_UP_PIN_PULSE_TIME_MS));

    buf = '1';
    ret = ::write(power_up_fd, &buf, sizeof(buf));
    if (ret < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "write error for setting 1 !");
        throw sdbusplus::xyz::openbmc_project::Chassis::Common::Error::
            IOError();
    }

    state(newState);
    return 0;
}

int32_t PowerControl::getPowerState()
{
    return state();
}
