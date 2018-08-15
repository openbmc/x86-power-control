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

#include <experimental/filesystem>
#include <fcntl.h>
#include <fstream>
#include <phosphor-logging/elog-errors.hpp>
#include <unistd.h>
#include <xyz/openbmc_project/Common/error.hpp>
#include "gpio.hpp"

const static constexpr char *SYSMGR_SERVICE = "org.openbmc.managers.System";
const static constexpr char *SYSMGR_OBJ_PATH = "/org/openbmc/managers/System";
const static constexpr char *SYSMGR_INTERFACE = "org.openbmc.managers.System";

int closeGpio(int fd)
{
    if (fd > 0)
    {
        ::close(fd);
    }
    return 0;
}

int configGpio(const char *gpioName, int *fd, sdbusplus::bus::bus &bus)
{
    sdbusplus::message::message method = bus.new_method_call(
        SYSMGR_SERVICE, SYSMGR_OBJ_PATH, SYSMGR_INTERFACE, "gpioInit");

    method.append(gpioName);

    sdbusplus::message::message result = bus.call(method);

    if (result.is_method_error())
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "configGPIO: bus call error!");
        return -1;
    }

    int32_t gpioNum = -1;
    std::string gpioDev;
    std::string gpioDirection;

    result.read(gpioDev, gpioNum, gpioDirection);

    if (gpioDev.empty())
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "configGPIO: gpioDev error!");
        return -1;
    }

    if (gpioDirection.empty())
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "configGPIO: gpioDirection error!");
        return -1;
    }

    std::fstream stream;

    stream.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    std::string devPath =
        gpioDev + "/gpio" + std::to_string(gpioNum) + "/value";

    std::experimental::filesystem::path fullPath(devPath);

    if (std::experimental::filesystem::exists(fullPath))
    {
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "GPIO exported",
            phosphor::logging::entry("PATH=%s", devPath.c_str()));
    }
    else
    {
        devPath = gpioDev + "/export";

        stream.open(devPath, std::fstream::out);

        if (!stream.good())
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Error in opening for write!",
                phosphor::logging::entry("PATH=%s", devPath.c_str()),
                phosphor::logging::entry("NUM=%d", gpioNum));
            return -1;
        }

        stream << gpioNum;
        stream.close();
    }

    if (gpioDirection == "out")
    {
        devPath = gpioDev + "/gpio" + std::to_string(gpioNum) + "/value";

        uint32_t currentValue = 0;

        stream.open(devPath, std::fstream::in);

        if (!stream.good())
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Error in opening for read!",
                phosphor::logging::entry("PATH=%s", devPath.c_str()));
            return -1;
        }

        stream >> currentValue;
        stream.close();

        const char *direction = currentValue ? "high" : "low";

        devPath = gpioDev + "/gpio" + std::to_string(gpioNum) + "/direction";

        stream.open(devPath, std::fstream::out);

        if (!stream.good())
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Error in opening for write!");
            return -1;
        }

        stream << direction;
        stream.close();
    }
    else if (gpioDirection == "in")
    {
        devPath = gpioDev + "/gpio" + std::to_string(gpioNum) + "/direction";

        stream.open(devPath, std::fstream::out);

        if (!stream.good())
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Error in opening for write!");
            return -1;
        }

        stream << gpioDirection;
        stream.close();
    }
    else if (gpioDirection == "both")
    {

        // For gpio configured as ‘both’, it is an interrupt pin and trigged on
        // both rising and falling signals
        devPath = gpioDev + "/gpio" + std::to_string(gpioNum) + "/edge";

        stream.open(devPath, std::fstream::out);

        if (!stream.good())
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Error in opening for write!");
            return -1;
        }

        stream << gpioDirection;
        stream.close();
    }

    devPath = gpioDev + "/gpio" + std::to_string(gpioNum) + "/value";

    *fd = ::open(devPath.c_str(), O_RDWR | O_NONBLOCK);

    if (*fd < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("open error!");
        return -1;
    }

    return 0;
}
