/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#pragma once

#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <nlohmann/json.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <filesystem>
#include <string_view>

namespace power_control
{

/**
 * @brief Persistent State Manager
 *
 * This manager supposed to store runtime parameters that supposed to be
 * persistent over BMC reboot. It provides simple Get/Set interface and handle
 * default values, hardcoded in getDefault() method.
 * @note: currently only string parameters supported
 */
using dbusPropertiesList =
    boost::container::flat_map<std::string,
                               std::variant<std::string, uint64_t>>;
/**
 * @brief The class contains functions to invoke power restore policy.
 *
 * This class only exists to unite all PowerRestore-related code. It supposed
 * to run only once on application startup.
 */
class PowerRestoreController
{
  public:
    PowerRestoreController(boost::asio::io_service& io) :
        policyInvoked(false), powerRestoreDelay(-1), powerRestoreTimer(io),
        timerFired(false)
    {}
    /**
     * @brief Power Restore entry point.
     *
     * Call this to start Power Restore algorithm.
     */
    void run();
    /**
     * @brief Initialize configuration parameters.
     *
     * Parse list of properties, received from dbus, to set Power Restore
     * algorithm configuration.
     * @param props - map of property names and values
     */
    void setProperties(const dbusPropertiesList& props);

  private:

    bool policyInvoked;
    std::string powerRestorePolicy;
    int powerRestoreDelay;
    std::list<sdbusplus::bus::match::match> matches;
    boost::asio::steady_timer powerRestoreTimer;
    bool timerFired;
#ifdef USE_ACBOOT
    std::string acBoot;
#endif // USE_ACBOOT

    /**
     * @brief Check if all required algorithms parameters are set
     *
     * Call this after set any of Power Restore algorithm parameters. Once all
     * parameters are set this will run invoke() function.
     */
    void invokeIfReady();
    /**
     * @brief Actually perform power restore actions.
     *
     * Take Power Restore actions according to Policy and other parameters.
     */
    void invoke();
    /**
     * @brief Check if power was dropped.
     *
     * Read last saved power state to determine if host power was enabled before
     * last BMC reboot.
     */
    bool wasPowerDropped();
};

class PersistentState
{
  public:
    /**
     * List of all supported parameters
     */
    enum class Params
    {
        PowerState,
    };

    /**
     * @brief Persistent storage initialization
     *
     * Class constructor automatically load last state from JSON file
     */
    PersistentState();
    /**
     * @brief Persistent storage cleanup
     *
     * Class destructor automatically save state to JSON file
     */
    ~PersistentState();
    /**
     * @brief Get parameter value from the storage
     *
     * Get the parameter from cached storage. Default value returned, if
     * parameter was not set before.
     * @param parameter - parameter to get
     * @return parameter value
     */
    const std::string get(Params parameter);
    /**
     * @brief Store parameter value
     *
     * Set the parameter value in cached storage and dump it to disk.
     * @param parameter - parameter to set
     * @param value - parameter value to assign
     */
    void set(Params parameter, const std::string& value);

  private:
    nlohmann::json stateData;
    const std::filesystem::path powerControlDir = "/var/lib/power-control";
    const std::string_view stateFile = "state.json";
    const int indentationSize = 2;

    /**
     * @brief Covert parameter ID to name
     *
     * Get the name corresponding to the given parameter.
     * String name only used by the manager internal to generate human-readable
     * JSON.
     * @param parameter - parameter to convert
     * @return parameter name
     */
    const std::string getName(const Params parameter);
    /**
     * @brief Get default parameter value
     *
     * Get the default value, associated with given parameter.
     * @param parameter - parameter to get
     * @return parameter default value
     */
    const std::string getDefault(const Params parameter);
    /**
     * @brief Save cache to file on disk
     */
    void saveState();
};

} // namespace power_control
