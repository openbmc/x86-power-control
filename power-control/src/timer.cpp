/**
 * Copyright © 2017 IBM Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "timer.hpp"
#include <systemd/sd-event.h>
#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/log.hpp>
#include <xyz/openbmc_project/Common/error.hpp>
#include <chrono>
namespace phosphor
{
namespace watchdog
{

// For throwing exception
using namespace phosphor::logging;
using InternalFailure =
    sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure;

// Initializes the timer object
void Timer::initialize()
{
    // This can not be called more than once.
    if (eventSource.get())
    {
        log<level::ERR>("Timer already initialized");
        elog<InternalFailure>();
    }

    // Add infinite expiration time
    decltype(eventSource.get()) sourcePtr = nullptr;
    auto r = sd_event_add_time(event.get(), &sourcePtr,
                               CLOCK_MONOTONIC, // Time base
                               UINT64_MAX,      // Expire time - way long time
                               0,               // Use default event accuracy
                               timeoutHandler,  // Callback handler on timeout
                               this);           // User data
    eventSource.reset(sourcePtr);

    if (r < 0)
    {
        log<level::ERR>("Timer initialization failed");
        elog<InternalFailure>();
    }

    // Disable the timer for now
    setEnabled<std::false_type>();
}

// callback handler on timeout
int Timer::timeoutHandler(sd_event_source* eventSource, uint64_t usec,
                          void* userData)
{
    using namespace phosphor::logging;

    auto timer = static_cast<Timer*>(userData);
    timer->expire = true;

    // Call an optional callback function
    if (timer->userCallBack)
    {
        timer->userCallBack();
    }
    return 0;
}

// Gets the time from steady_clock
std::chrono::microseconds Timer::getCurrentTime()
{
    using namespace std::chrono;
    auto usec = steady_clock::now().time_since_epoch();
    return duration_cast<microseconds>(usec);
}

// Sets the expiration time and arms the timer
void Timer::start(std::chrono::microseconds usec)
{
    using namespace std::chrono;

    // Get the current MONOTONIC time and add the delta
    auto expireTime = getCurrentTime() + usec;

    // Set the time
    auto r = sd_event_source_set_time(eventSource.get(), expireTime.count());
    if (r < 0)
    {
        log<level::ERR>(
            "Error setting the expiration time",
            entry("MSEC=%llu", duration_cast<milliseconds>(usec).count()));
        elog<InternalFailure>();
    }
}

// Returns current timer enablement type
int Timer::getEnabled() const
{
    int enabled{};
    auto r = sd_event_source_get_enabled(eventSource.get(), &enabled);
    if (r < 0)
    {
        log<level::ERR>("Error geting current timer type enablement state");
        elog<InternalFailure>();
    }
    return enabled;
}

// Enables / disables the timer
void Timer::setEnabled(int type)
{
    auto r = sd_event_source_set_enabled(eventSource.get(), type);
    if (r < 0)
    {
        log<level::ERR>("Error setting the timer type", entry("TYPE=%d", type));
        elog<InternalFailure>();
    }
}

// Returns time remaining before expiration
std::chrono::microseconds Timer::getRemaining() const
{
    uint64_t next = 0;
    auto r = sd_event_source_get_time(eventSource.get(), &next);
    if (r < 0)
    {
        log<level::ERR>("Error fetching remaining time to expire");
        elog<InternalFailure>();
    }
    return std::chrono::microseconds(next);
}

} // namespace watchdog
} // namespace phosphor
