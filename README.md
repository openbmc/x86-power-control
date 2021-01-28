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
server platforms, such as S2600WFT.  It is likely that other platforms will work
as well.

Because this relies on the hardware passthrough support in the AST2500 to
function, it requires a few patches to work correctly.

This patch adds support to UBOOT to keep the passthrough enabled
https://github.com/Intel-BMC/openbmc/blob/intel/meta-openbmc-mods/meta-common/
recipes-bsp/u-boot/files/0005-enable-passthrough-in-uboot.patch

The DTS file for your platform will need the following GPIO definitions
RESET_BUTTON
RESET_OUT
POWER_BUTTON
POWER_OUT

On an aspeed, these are generally connected to E0, E1, E2, and E3 respectively.
An example of this is available in the s2600WF config.

This patch allows the passthrough to be reenabled to the default condition when
the appropriate pin is released.  This allows power control to take control
when needed by a user power action, but leave the hardware in control a majority
of the time, reducing the possibility of bricking a system due to a failed BMC.

https://github.com/Intel-BMC/openbmc/blob/intel/meta-openbmc-mods/meta-ast2500/recipes-kernel/linux/linux-aspeed/0002-Enable-pass-through-on-GPIOE1-and-GPIOE3-free.patch
https://github.com/Intel-BMC/openbmc/blob/intel/meta-openbmc-mods/meta-ast2500/recipes-kernel/linux/linux-aspeed/0003-Enable-GPIOE0-and-GPIOE2-pass-through-by-default.patch
https://github.com/Intel-BMC/openbmc/blob/intel/meta-openbmc-mods/meta-ast2500/recipes-kernel/linux/linux-aspeed/0006-Allow-monitoring-of-power-control-input-GPIOs.patch


Caveats:
This implementation does not currently implement the common targets that other
implementations do.  There were several attempts to, but all ended in timing
issues and boot inconsistencies during stress operations.

## Build Options

#### REMOVE_POST_COMPLETE
The POST Complete GPIO is usually held asserted by BIOS after POST complete
and de-asserts on reset.  Based on this behavior, it can be used to detect both

1. When BIOS completes and the OperatingSystemState can change to "Standby".
2. When a warm reset occurs.

Some systems are removing the POST Complete GPIO and replacing it with two
alternative signals.  When this option is enabled, x86-power-control will stop
monitoring POST Complete and will switch to the following signals:

1. CoreBiosDone: Sent from BIOS through a mailbox to indicate when BIOS
   completes and the OperatingSystemState can change to "Standby".
2. PLT_RST: Sent over eSPI to indicate when the platform is in reset and can
   be used to detect warm reset.