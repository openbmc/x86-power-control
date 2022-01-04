/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#pragma once

#include <nlohmann/json.hpp>

#include <filesystem>
#include <string_view>

namespace power_control
{

class PersistentState
{
  public:
    enum class Params
    {
        PowerState,
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
