/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Ivenix Maryann board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6QMARYANN_CONFIG_H
#define __MX6QMARYANN_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

#define CONFIG_MACH_TYPE	4419
#define CONFIG_MXC_UART_BASE	UART5_BASE
#define CONFIG_CONSOLE_DEV	"ttymxc4"
#define CONFIG_MMCROOT		"/dev/mmcblk0p1"
#define CONFIG_DEFAULT_FDT_FILE	"imx6q-ginger.dtb"
#define PHYS_SDRAM_SIZE		(1u * 1024 * 1024 * 1024)

#include "mx6maryann_common.h"

#define CONFIG_SYS_FSL_USDHC_NUM	1
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_SYS_MMC_ENV_DEV		0
#endif

#define CONFIG_STANDALONE_LOAD_ADDR 0x12000000

#endif                         /* __MX6QMARYANN_CONFIG_H */
