# X86 power control

This repository contains an OpenBMC compliant implementation of power control
for x86 servers. It relies on a number of features to do its job. It has several
intentional design goals.

1. The BMC should maintain the Host state machine internally, and be able to
   track state changes.
2. The implementation should either give the requested power control result, or
   should log an error on the failure it detected.
3. The BMC should support all the common operations, hard power on/off/cycle,
   soft power on/off/cycle.

This daemon has been successfully used on a variety of server platforms; it
should be able to support platforms with power control GPIOs similar to those in
its config file.

x86-power-control uses default json file (power-config-host0.json) for GPIO
configuration. However this can be customized by producing your own
power-config-host0.json file.

## Signal Definitions

Definitions can be configured by two type: GPIO and DBUS

### GPIO

For the platform having direct GPIO access can use the type GPIO and define like
below.

```json
{
  "Name": "PostComplete",
  "LineName": "POST_COMPLETE",
  "Type": "GPIO",
  "Polarity": "ActiveLow"
}
```

### DBUS

For the platform not having direct GPIO access can use dbus based event monitor
by using the type DBUS.

```json
{
  "Name": "PowerButton",
  "DbusName": "xyz.openbmc_project.Chassis.Event",
  "Path": "/xyz/openbmc_project/Chassis/Event",
  "Interface": "xyz.openbmc_project.Chassis.Event",
  "Property": "PowerButton_Host1",
  "Polarity": "ActiveLow",
  "Type": "DBUS"
}
```

x86-power-control will monitor the property change from the given DbusName and
take appropriate action. \*define Property as a bool variable.

Caveats: This implementation does not currently implement the common targets
that other implementations do. There were several attempts to, but all ended in
timing issues and boot inconsistencies during stress operations.

## Build Options

### chassis-system-reset

Enable chassis system power reset to allow removing power and restoring back.

### use-plt-rst

The POST Complete GPIO is usually held asserted by BIOS after POST complete and
de-asserts on reset. This de-assert behavior is currently used to detect warm
resets.

Some systems are adding support for a PLT_RST eSPI signal that can be used to
more accurately detect warm resets. When this option is enabled,
x86-power-control will use PLT_RST to detect warm resets instead of POST
Complete.

See <https://github.com/Intel-BMC/host-misc-comm-manager> for implementation
example.
