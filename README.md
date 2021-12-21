# X86 power control

This repository contains an OpenBMC compliant implementation of power control
for x86 servers.  It relies on a number of features to do its job.  It has
several intentional design goals.
1. The BMC should maintain the Host state machine internally, and be able to
   track state changes.
2. The implementation should either give the requested power control result, or
   should log an error on the failure it detected.
3. The BMC should support all the common operations, hard power on/off/cycle,
   soft power on/off/cycle.

At this point in time, this daemon targets Lewisburg based, dual socket x86
server platforms, such as S2600WFT.  It is likely that other platforms will
work as well.

The DTS file for your platform will need the following GPIO definitions (not
all of them are necessary):
- id-button
- nmi-button
- host-nmi-control
- host-ready
- power-button
- power-chassis-good
- power-chassis-control
- reset-button
- host-reset-control
- host-sio-on-control
- host-sio-pwr-good
- host-sio-s5

x86-power-control uses default json file (power-config-host0.json) for GPIO
configuration.  However this can be customized by producing your own
power-config-host0.json file.

Definitions can be configured by two type:

1. GPIO

For the platform having direct GPIO access can use the type GPIO and define
like below.

    {
        "Name" : "PostComplete",
        "LineName" : "host-ready",
        "Type" : "GPIO"
    },

2. DBUS

For the platform not having direct GPIO access can use dbus based event monitor
by using the type DBUS.

    {
        "Name" : "PowerButton",
        "DbusName" : "xyz.openbmc_project.Chassis.Event",
        "Path" : "/xyz/openbmc_project/Chassis/Event",
        "Interface" : "xyz.openbmc_project.Chassis.Event",
        "Property" : "PowerButton_Host1",
        "Type" : "DBUS"
    },

x86-power-control will monitor the property change from the given DbusName and
take appropriate action.  *define Property as a bool variable.

Caveats:
This implementation does not currently implement the common targets that other
implementations do.  There were several attempts to, but all ended in timing
issues and boot inconsistencies during stress operations.

## Build Options

#### chassis-system-reset
Enable chassis system power reset to allow removing power and restoring back.

#### use-plt-rst
The POST Complete GPIO is usually held asserted by BIOS after POST complete and
de-asserts on reset.  This de-assert behavior is currently used to detect warm
resets.

Some systems are adding support for a PLT_RST eSPI signal that can be used to
more accurately detect warm resets.  When this option is enabled,
x86-power-control will use PLT_RST to detect warm resets instead of POST
Complete.

See https://github.com/Intel-BMC/host-misc-comm-manager for implementation
example.
