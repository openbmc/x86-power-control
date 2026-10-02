/*
// Copyright (c) 2026 Intel Corporation
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

#include "config_parser.hpp"

#include <phosphor-logging/lg2.hpp>

#include <fstream>
#include <string>

namespace power_control
{

namespace
{

enum class DbusConfigType
{
    name = 1,
    path,
    interface,
    property
};

// Mandatory config parameters for dbus inputs
const boost::container::flat_map<DbusConfigType, std::string> dbusParams = {
    {DbusConfigType::name, "DbusName"},
    {DbusConfigType::path, "Path"},
    {DbusConfigType::interface, "Interface"},
    {DbusConfigType::property, "Property"}};

bool parseGPIOConfig(ConfigData& configData, const nlohmann::json& gpioConfig)
{
    auto lineNameIt = gpioConfig.find("LineName");
    if (lineNameIt == gpioConfig.end())
    {
        lg2::error(
            "The \'LineName\' field must be defined for GPIO configuration");
        return false;
    }
    const std::string* lineName = lineNameIt->get_ptr<const std::string*>();
    if (lineName == nullptr)
    {
        lg2::error("The \'LineName\' field must be a string");
        return false;
    }
    configData.lineName = *lineName;

    auto polarityIt = gpioConfig.find("Polarity");
    if (polarityIt == gpioConfig.end())
    {
        lg2::error("Polarity field not found for {GPIO_NAME}", "GPIO_NAME",
                   configData.lineName);
        return false;
    }
    const std::string* polarity = polarityIt->get_ptr<const std::string*>();
    if (polarity == nullptr)
    {
        lg2::error("The \'Polarity\' field must be a string");
        return false;
    }
    if (*polarity == "ActiveLow")
    {
        configData.polarity = false;
    }
    else if (*polarity == "ActiveHigh")
    {
        configData.polarity = true;
    }
    else
    {
        lg2::error(
            "Polarity defined but not properly setup. Please only ActiveHigh or ActiveLow. Currently set to {POLARITY}",
            "POLARITY", *polarity);
        return false;
    }
    return true;
}

bool parseDBUSConfig(ConfigData& configData, const nlohmann::json& gpioConfig,
                     const std::string& gpioName)
{
    // if dbus based gpio config is defined read and update the dbus
    // params corresponding to the gpio config instance
    for (auto& [key, dbusParamName] : dbusParams)
    {
        auto it = gpioConfig.find(dbusParamName);
        if (it == gpioConfig.end())
        {
            lg2::error(
                "The {DBUS_NAME} field must be defined for Dbus configuration ",
                "DBUS_NAME", dbusParamName);
            return false;
        }
        const std::string* val = it->get_ptr<const std::string*>();
        if (val == nullptr)
        {
            lg2::error("The {DBUS_NAME} field must be a string", "DBUS_NAME",
                       dbusParamName);
            return false;
        }
        switch (key)
        {
            case DbusConfigType::name:
                configData.dbusName = *val;
                break;
            case DbusConfigType::path:
                configData.path = *val;
                break;
            case DbusConfigType::interface:
                configData.interface = *val;
                break;
            case DbusConfigType::property:
                configData.lineName = *val;
                break;
        }
    }

    // dbus-based inputs must be active-high.
    configData.polarity = true;

    // MatchRegex is optional
    auto item = gpioConfig.find("MatchRegex");
    if (item != gpioConfig.end())
    {
        const std::string* regexStr = item->get_ptr<const std::string*>();
        if (regexStr == nullptr)
        {
            lg2::error("MatchRegex for {NAME} must be a string", "NAME",
                       gpioName);
            return false;
        }
        try
        {
            configData.matchRegex = std::regex(*regexStr);
        }
        catch (const std::regex_error& e)
        {
            lg2::error("Invalid MatchRegex for {NAME}: {ERR}", "NAME", gpioName,
                       "ERR", e.what());
            return false;
        }
    }
    return true;
}

void parseTimerConfig(const nlohmann::json& timers,
                      boost::container::flat_map<std::string, int>& timerMap)
{
    // read and store the timer values from json config to Timer Map
    for (auto& [key, timerValue] : timerMap)
    {
        timerValue = timers.value(key, timerValue);
    }
}

void parseEventConfig(const nlohmann::json& jsonData,
                      boost::container::flat_map<std::string, bool>& eventMap)
{
    auto events = jsonData.find("event_configs");
    if (events == jsonData.end() || !events->is_object())
    {
        return;
    }
    // read and store the event values from json config to event config map
    for (auto& [key, value] : eventMap)
    {
        value = events->value(key, value);
    }
}

} // namespace

int loadConfigValues(
    const std::string& node,
    const boost::container::flat_map<std::string, ConfigData*>& powerSignalMap,
    boost::container::flat_map<std::string, int>& timerMap,
    boost::container::flat_map<std::string, bool>& eventConfigMap)
{
    const std::string configFilePath =
        "/usr/share/x86-power-control/power-config-host" + node + ".json";
    std::ifstream configFile(configFilePath.c_str());
    if (!configFile.is_open())
    {
        lg2::error("loadConfigValues: Cannot open config path \'{PATH}\'",
                   "PATH", configFilePath);
        return -1;
    }
    auto jsonData = nlohmann::json::parse(configFile, nullptr, true, true);

    if (jsonData.is_discarded())
    {
        lg2::error("Power config readings JSON parser failure");
        return -1;
    }

    for (nlohmann::json& gpioConfig : jsonData["gpio_configs"])
    {
        auto nameIt = gpioConfig.find("Name");
        if (nameIt == gpioConfig.end())
        {
            lg2::error("The 'Name' field must be defined in Json file");
            return -1;
        }

        // Iterate through the powersignal map to check if the gpio json config
        // entry is valid
        const std::string* namePtr = nameIt->get_ptr<const std::string*>();
        if (namePtr == nullptr)
        {
            lg2::error("The 'Name' field must be a string");
            return -1;
        }
        std::string gpioName = *namePtr;
        auto signalMapIter = powerSignalMap.find(gpioName);
        if (signalMapIter == powerSignalMap.end())
        {
            lg2::error(
                "{GPIO_NAME} is not a recognized power-control signal name",
                "GPIO_NAME", gpioName);
            return -1;
        }

        // assign the power signal name to the corresponding structure reference
        // from map then fillup the structure with coressponding json config
        // value
        ConfigData* tempGpioData = signalMapIter->second;
        tempGpioData->name = gpioName;

        auto typeIt = gpioConfig.find("Type");
        if (typeIt == gpioConfig.end())
        {
            lg2::error("The \'Type\' field must be defined in Json file");
            return -1;
        }

        const std::string* typePtr = typeIt->get_ptr<const std::string*>();
        if (typePtr == nullptr)
        {
            lg2::error("The \'Type\' field must be a string");
            return -1;
        }
        std::string signalType = *typePtr;
        if (signalType == "GPIO")
        {
            tempGpioData->type = ConfigType::GPIO;
        }
        else if (signalType == "DBUS")
        {
            tempGpioData->type = ConfigType::DBUS;
        }
        else
        {
            lg2::error("{TYPE} is not a recognized power-control signal type",
                       "TYPE", signalType);
            return -1;
        }

        if (tempGpioData->type == ConfigType::GPIO)
        {
            if (!parseGPIOConfig(*tempGpioData, gpioConfig))
            {
                return -1;
            }
        }
        else
        {
            if (!parseDBUSConfig(*tempGpioData, gpioConfig, gpioName))
            {
                return -1;
            }
        }
    }

    parseTimerConfig(jsonData["timing_configs"], timerMap);
    // optional section
    parseEventConfig(jsonData, eventConfigMap);

    return 0;
}

} // namespace power_control
