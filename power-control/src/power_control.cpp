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

    state(newState);

    ret = ::lseek(power_up_fd, 0, SEEK_SET);
    if (ret < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("lseek error!");
        throw sdbusplus::xyz::openbmc_project::Chassis::Common::Error::
            IOError();
    }

    /*
    To simulate a button click pulse (pressed and release).
    */
    buf = '0';
    ret = ::write(power_up_fd, &buf, sizeof(buf));
    if (ret < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("write error!");
        throw sdbusplus::xyz::openbmc_project::Chassis::Common::Error::
            IOError();
    }

    if (1 == newState)
    {
        phosphor::logging::log<phosphor::logging::level::DEBUG>(
            "setPowerState power on");
        std::this_thread::sleep_for(
            std::chrono::milliseconds(POWER_ON_PULSE_TIME_MS));
    }
    else
    {
        phosphor::logging::log<phosphor::logging::level::DEBUG>(
            "setPowerState power off");
        std::this_thread::sleep_for(
            std::chrono::milliseconds(POWER_OFF_PULSE_TIME_MS));
    }

    buf = '1';
    ret = ::write(power_up_fd, &buf, sizeof(buf));
    if (ret < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("write error!");
        throw sdbusplus::xyz::openbmc_project::Chassis::Common::Error::
            IOError();
    }

    return 0;
}

int32_t PowerControl::getPowerState()
{
    return state();
}
