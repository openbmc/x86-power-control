// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright 2026 Intel Corporation

#include "config_parser.hpp"

#include <boost/container/flat_map.hpp>
#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <regex>
#include <string>
#include <system_error>

#include <gtest/gtest.h>

using namespace power_control;

class TempConfigFile
{
  public:
    explicit TempConfigFile(const std::string& content)
    {
        static int counter = 0;
        filePath =
            std::filesystem::temp_directory_path() /
            ("config_parser_test_" + std::to_string(counter++) + ".json");
        std::ofstream out(filePath);
        out << content;
    }

    TempConfigFile(const TempConfigFile&) = delete;
    TempConfigFile& operator=(const TempConfigFile&) = delete;

    ~TempConfigFile()
    {
        std::error_code ec;
        std::filesystem::remove(filePath, ec);
    }

    const std::filesystem::path& path() const
    {
        return filePath;
    }

  private:
    std::filesystem::path filePath;
};

static nlohmann::json validGpioEntry()
{
    return nlohmann::json::parse(R"({
        "Name": "PowerOut",
        "Type": "GPIO",
        "LineName": "POWER_OUT",
        "Polarity": "ActiveHigh"
    })");
}

static nlohmann::json validDbusEntry()
{
    return nlohmann::json::parse(R"({
        "Name": "PowerOk",
        "Type": "DBUS",
        "DbusName": "xyz.openbmc_project.Example",
        "Path": "/xyz/openbmc_project/example",
        "Interface": "xyz.openbmc_project.Example.Value",
        "Property": "PowerOk"
    })");
}

class ConfigParserTest : public ::testing::Test
{
  protected:
    ConfigData powerOut;
    ConfigData powerOk;
    ConfigData postComplete;

    boost::container::flat_map<std::string, ConfigData*> signalMap = {
        {"PowerOut", &powerOut},
        {"PowerOk", &powerOk},
        {"PostComplete", &postComplete},
    };
    boost::container::flat_map<std::string, int> timerMap = {
        {"PowerPulseMs", 200},
        {"PowerCycleMs", 5000},
    };
    boost::container::flat_map<std::string, bool> eventMap = {
        {"NMIWhenPoweredOff", true},
    };

    int parse(const nlohmann::json& data)
    {
        TempConfigFile config(data.dump());
        return loadConfigValues(config.path(), signalMap, timerMap, eventMap);
    }

    int parseSignal(const nlohmann::json& entry)
    {
        nlohmann::json data;
        data["gpio_configs"].push_back(entry);
        return parse(data);
    }
};

TEST_F(ConfigParserTest, EmptyConfigKeepsDefaults)
{
    EXPECT_EQ(parse(nlohmann::json::object()), 0);
    EXPECT_EQ(timerMap["PowerPulseMs"], 200);
    EXPECT_TRUE(eventMap["NMIWhenPoweredOff"]);
}

TEST_F(ConfigParserTest, TimerOverridesDefault)
{
    nlohmann::json data =
        nlohmann::json::parse(R"({"timing_configs": {"PowerPulseMs": 333}})");
    EXPECT_EQ(parse(data), 0);
    EXPECT_EQ(timerMap["PowerPulseMs"], 333);
    EXPECT_EQ(timerMap["PowerCycleMs"], 5000);
}

TEST_F(ConfigParserTest, NegativeTimerValue)
{
    nlohmann::json data =
        nlohmann::json::parse(R"({"timing_configs": {"PowerPulseMs": -333}})");
    EXPECT_EQ(parse(data), -1);
}

TEST_F(ConfigParserTest, EventConfigOverridesDefault)
{
    nlohmann::json data = nlohmann::json::parse(
        R"({"event_configs": {"NMIWhenPoweredOff": false}})");
    EXPECT_EQ(parse(data), 0);
    EXPECT_FALSE(eventMap["NMIWhenPoweredOff"]);
}

TEST_F(ConfigParserTest, MissingConfigFile)
{
    EXPECT_EQ(loadConfigValues("/nonexistent/config_parser_test.json",
                               signalMap, timerMap, eventMap),
              -1);
}

TEST_F(ConfigParserTest, MissingNameFails)
{
    nlohmann::json entry = validGpioEntry();
    entry.erase("Name");
    EXPECT_EQ(parseSignal(entry), -1);
}

TEST_F(ConfigParserTest, MissingTypeFails)
{
    nlohmann::json entry = validGpioEntry();
    entry.erase("Type");
    EXPECT_EQ(parseSignal(entry), -1);
}

TEST_F(ConfigParserTest, InvalidTypeFails)
{
    nlohmann::json entry = validGpioEntry();
    entry["Type"] = "eSPI";
    EXPECT_EQ(parseSignal(entry), -1);
}

TEST_F(ConfigParserTest, UnknownSignalNameFails)
{
    nlohmann::json entry = validGpioEntry();
    entry["Name"] = "FakeSignalName";
    EXPECT_EQ(parseSignal(entry), -1);
}

class GpioConfigTest : public ConfigParserTest
{};

TEST_F(GpioConfigTest, ParsesActiveHigh)
{
    EXPECT_EQ(parseSignal(validGpioEntry()), 0);
    EXPECT_EQ(powerOut.name, "PowerOut");
    EXPECT_EQ(powerOut.lineName, "POWER_OUT");
    EXPECT_EQ(powerOut.type, ConfigType::GPIO);
    EXPECT_TRUE(powerOut.polarity);
}

TEST_F(GpioConfigTest, ParsesActiveLow)
{
    nlohmann::json entry = validGpioEntry();
    entry["Name"] = "PowerOk";
    entry["LineName"] = "POWER_OK";
    entry["Polarity"] = "ActiveLow";
    EXPECT_EQ(parseSignal(entry), 0);
    EXPECT_FALSE(powerOk.polarity);
}

TEST_F(GpioConfigTest, InvalidPolarityFails)
{
    nlohmann::json entry = validGpioEntry();
    entry["Polarity"] = "Sideways";
    EXPECT_EQ(parseSignal(entry), -1);
}

class GpioMissingFieldTest :
    public ConfigParserTest,
    public ::testing::WithParamInterface<std::string>
{};

TEST_P(GpioMissingFieldTest, MissingRequiredFieldFails)
{
    nlohmann::json entry = validGpioEntry();
    entry.erase(GetParam());
    EXPECT_EQ(parseSignal(entry), -1);
}

INSTANTIATE_TEST_SUITE_P(RequiredFields, GpioMissingFieldTest,
                         ::testing::Values("LineName", "Polarity"));

class DbusConfigTest : public ConfigParserTest
{};

TEST_F(DbusConfigTest, ParsesAllFields)
{
    EXPECT_EQ(parseSignal(validDbusEntry()), 0);
    EXPECT_EQ(powerOk.type, ConfigType::DBUS);
    EXPECT_EQ(powerOk.dbusName, "xyz.openbmc_project.Example");
    EXPECT_EQ(powerOk.path, "/xyz/openbmc_project/example");
    EXPECT_EQ(powerOk.interface, "xyz.openbmc_project.Example.Value");
    // The "Property" field is stored in lineName.
    EXPECT_EQ(powerOk.lineName, "PowerOk");
    // The "Polarity" filed is forced active-high.
    EXPECT_TRUE(powerOk.polarity);
    EXPECT_FALSE(powerOk.matchRegex.has_value());
}

TEST_F(DbusConfigTest, ParsesMatchRegex)
{
    nlohmann::json entry = validDbusEntry();
    entry["Name"] = "PostComplete";
    entry["Property"] = "Status";
    entry["MatchRegex"] = "^Completed$";
    EXPECT_EQ(parseSignal(entry), 0);
    ASSERT_TRUE(postComplete.matchRegex.has_value());
    // clang-tidy bugprone-unchecked-optional-access check
    if (postComplete.matchRegex.has_value())
    {
        EXPECT_TRUE(
            std::regex_match("Completed", postComplete.matchRegex.value()));
    }
}

TEST_F(DbusConfigTest, InvalidMatchRegexFails)
{
    nlohmann::json entry = validDbusEntry();
    entry["Name"] = "PostComplete";
    entry["Property"] = "Status";
    entry["MatchRegex"] = "[";
    EXPECT_EQ(parseSignal(entry), -1);
}

TEST_F(DbusConfigTest, NonStringMatchRegexFails)
{
    nlohmann::json entry = validDbusEntry();
    entry["Name"] = "PostComplete";
    entry["Property"] = "Status";
    entry["MatchRegex"] = 123;
    EXPECT_EQ(parseSignal(entry), -1);
}

class DbusMissingFieldTest :
    public ConfigParserTest,
    public ::testing::WithParamInterface<std::string>
{};

TEST_P(DbusMissingFieldTest, MissingRequiredFieldFails)
{
    nlohmann::json entry = validDbusEntry();
    entry.erase(GetParam());
    EXPECT_EQ(parseSignal(entry), -1);
}

INSTANTIATE_TEST_SUITE_P(RequiredFields, DbusMissingFieldTest,
                         ::testing::Values("DbusName", "Path", "Interface",
                                           "Property"));
