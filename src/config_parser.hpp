// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright 2026 Intel Corporation

#pragma once

#include <boost/container/flat_map.hpp>
#include <nlohmann/json.hpp>

#include <filesystem>
#include <optional>
#include <regex>
#include <string>

namespace power_control
{

enum class ConfigType
{
    GPIO = 1,
    DBUS
};

struct ConfigData
{
    std::string name;
    std::string lineName;
    std::string dbusName;
    std::string path;
    std::string interface;
    std::optional<std::regex> matchRegex;
    bool polarity;
    ConfigType type;
};

/**
 * @brief Load and parse the power-control JSON configuration file.
 *
 * Opens @p configFile, parses its JSON content and populates the signal, timer
 * and event settings. The caller provides the path, so the file-location policy
 * stays with the caller and tests can point at their own file.
 *
 * @param configFile - path to the JSON configuration file
 * @param powerSignalMap - map of signal names to the ConfigData to populate
 * @param timerMap - timer values populated from the "timing_configs" section
 * @param eventConfigMap - values populated from the "event_configs" section
 * @return 0 on success, -1 on failure
 */
int loadConfigValues(
    const std::filesystem::path& configFile,
    const boost::container::flat_map<std::string, ConfigData*>& powerSignalMap,
    boost::container::flat_map<std::string, int>& timerMap,
    boost::container::flat_map<std::string, bool>& eventConfigMap);

} // namespace power_control
