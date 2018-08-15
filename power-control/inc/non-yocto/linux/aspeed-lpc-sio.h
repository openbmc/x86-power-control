/*
 * Copyright (C) 2012-2020	ASPEED Technology Inc.
 * Copyright (c) 2017 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 */

#ifndef _UAPI_LINUX_ASPEED_LPC_SIO_H
#define _UAPI_LINUX_ASPEED_LPC_SIO_H

#include <linux/ioctl.h>

enum ACPI_SLP_STATE
{
    ACPI_STATE_S12 = 1,
    ACPI_STATE_S3I,
    ACPI_STATE_S45
};

/* SWC & ACPI for SuperIO IOCTL */
enum SIO_CMD
{
    SIO_GET_ACPI_STATE = 0,
    SIO_GET_PWRGD_STATUS,
    SIO_GET_ONCTL_STATUS,
    SIO_SET_ONCTL_GPIO,
    SIO_GET_PWRBTN_OVERRIDE,
    SIO_GET_PFAIL_STATUS, /* Start from AC Loss */

    SIO_MAX_CMD
};

struct sio_ioctl_data
{
    unsigned short sio_cmd;
    unsigned short param;
    unsigned int data;
};

#define SIO_IOC_BASE 'P'
#define SIO_IOC_COMMAND _IOWR(SIO_IOC_BASE, 1, struct sio_ioctl_data)

#endif /* _UAPI_LINUX_ASPEED_LPC_SIO_H */
