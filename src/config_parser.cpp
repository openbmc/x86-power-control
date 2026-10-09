// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright 2026 Intel Corporation

#include "config_parser.hpp"

#include <phosphor-logging/lg2.hpp>

#include <filesystem>
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

bool parseTimerConfig(const nlohmann::json& timers,
                      boost::container::flat_map<std::string, int>& timerMap)
{
    // read and store the timer values from json config to Timer Map
    for (auto& [key, timerValue] : timerMap)
    {
        int value = timers.value(key, timerValue);
        if (value < 0)
        {
            lg2::error("Timer {NAME} must not be negative: {VALUE}", "NAME",
                       key, "VALUE", value);
            return false;
        }
        timerValue = value;
    }
    return true;
}

void parseEventConfig(const nlohmann::json& events,
                      boost::container::flat_map<std::string, bool>& eventMap)
{
    // read and store the event values from json config to event config map
    for (auto& [key, value] : eventMap)
    {
        value = events.value(key, value);
    }
}

bool parseGpioConfigs(
    const nlohmann::json& gpioConfigs,
    const boost::container::flat_map<std::string, ConfigData*>& powerSignalMap)
{
    for (const nlohmann::json& gpioConfig : gpioConfigs)
    {
        auto nameIt = gpioConfig.find("Name");
        if (nameIt == gpioConfig.end())
        {
            lg2::error("The 'Name' field must be defined in Json file");
            return false;
        }

        // Iterate through the powersignal map to check if the gpio json config
        // entry is valid
        const std::string* namePtr = nameIt->get_ptr<const std::string*>();
        if (namePtr == nullptr)
        {
            lg2::error("The 'Name' field must be a string");
            return false;
        }
        std::string gpioName = *namePtr;
        auto signalMapIter = powerSignalMap.find(gpioName);
        if (signalMapIter == powerSignalMap.end())
        {
            lg2::error(
                "{GPIO_NAME} is not a recognized power-control signal name",
                "GPIO_NAME", gpioName);
            return false;
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
            return false;
        }

        const std::string* typePtr = typeIt->get_ptr<const std::string*>();
        if (typePtr == nullptr)
        {
            lg2::error("The \'Type\' field must be a string");
            return false;
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
            return false;
        }

        if (tempGpioData->type == ConfigType::GPIO)
        {
            if (!parseGPIOConfig(*tempGpioData, gpioConfig))
            {
                return false;
            }
        }
        else
        {
            if (!parseDBUSConfig(*tempGpioData, gpioConfig, gpioName))
            {
                return false;
            }
        }
    }
    return true;
}

} // namespace

int loadConfigValues(
    const std::filesystem::path& configFile,
    const boost::container::flat_map<std::string, ConfigData*>& powerSignalMap,
    boost::container::flat_map<std::string, int>& timerMap,
    boost::container::flat_map<std::string, bool>& eventConfigMap)
{
    std::ifstream configStream(configFile);
    if (!configStream.is_open())
    {
        lg2::error("loadConfigValues: Cannot open config path \'{PATH}\'",
                   "PATH", configFile.string());
        return -1;
    }
    auto jsonData = nlohmann::json::parse(configStream, nullptr, true, true);
    if (jsonData.is_discarded())
    {
        lg2::error("Power config readings JSON parser failure");
        return -1;
    }

    auto gpioConfigs = jsonData.find("gpio_configs");
    if (gpioConfigs != jsonData.end())
    {
        if (!parseGpioConfigs(*gpioConfigs, powerSignalMap))
        {
            return -1;
        }
    }

    auto timingConfigs = jsonData.find("timing_configs");
    if (timingConfigs != jsonData.end())
    {
        if (!parseTimerConfig(*timingConfigs, timerMap))
        {
            return -1;
        }
    }

    auto eventConfigs = jsonData.find("event_configs");
    if (eventConfigs != jsonData.end())
    {
        parseEventConfig(*eventConfigs, eventConfigMap);
    }

    return 0;
}

} // namespace power_control
