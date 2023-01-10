/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6QMARYANN_COMMON_CONFIG_H
#define __MX6QMARYANN_COMMON_CONFIG_H

#define CONFIG_MX6

#include "mx6_common.h"
#include <asm/sizes.h>

#define CONFIG_SYS_GENERIC_BOARD
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_LDO_BYPASS_CHECK

#define CONFIG_MXC_UART

#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

/* MMC Configs */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR      0

#ifdef CONFIG_MMC
#	define CONFIG_CMD_MMC
#	define CONFIG_GENERIC_MMC
#	define CONFIG_BOUNCE_BUFFER
#	define CONFIG_CMD_EXT2
#	define CONFIG_DOS_PARTITION
#endif

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX              1
#define CONFIG_BAUDRATE                115200
#define CONFIG_SYS_BAUDRATE_TABLE      {9600, 19200, 38400, 57600, 115200}
#define CONFIG_SILENT_CONSOLE

#define CONFIG_AUTOBOOT_KEYED
#define CONFIG_AUTOBOOT_STOP_STR	" "
#define CONFIG_ZERO_BOOTDELAY_CHECK

#define CONFIG_SYS_NO_FLASH

/* Command definition */

#define CONFIG_CMD_BDI		/* bdinfo			*/
#define CONFIG_CMD_BOOTD	/* bootd			*/
#define CONFIG_CMD_CONSOLE	/* coninfo			*/
#define CONFIG_CMD_ECHO		/* echo arguments		*/
#define CONFIG_CMD_FUSE		/* Device fuse support		*/
#define CONFIG_CMD_ITEST	/* Integer (and string) test	*/

#define CONFIG_MP		/* cpu */

#ifndef CONFIG_SYS_NO_FLASH
#	define CONFIG_SYS_MAX_FLASH_SECT 4096
#	define CONFIG_SYS_MAX_FLASH_BANKS 1
#endif

#undef  CONFIG_MTD_DEBUG
#ifdef CONFIG_MTD_DEBUG
#define CONFIG_MTD_DEBUG_VERBOSE 10
#endif

#define CONFIG_CMD_NAND		/* NAND support			*/
#define CONFIG_CMD_NAND_LOCK_UNLOCK /* Enable lock/unlock support */
#define CONFIG_CMD_MTDPARTS	/* mtd parts support		*/
#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS

#define CONFIG_CMD_UBI		/* UBI Support			*/

#ifdef CONFIG_CMD_UBI
#	define CONFIG_RBTREE
#endif

#ifndef CONFIG_COMPACT
#	define CONFIG_CMD_EDITENV	/* editenv			*/
#	define CONFIG_CMD_IMI		/* iminfo			*/
#	define CONFIG_LZO
#	define CONFIG_CMD_UBIFS
#	define CONFIG_CMD_LOADB	/* loadb			*/
#	define CONFIG_CMD_MEMORY	/* md mm nm mw cp cmp crc base loop */
#	define CONFIG_CMD_MISC		/* Misc functions like sleep etc*/
#	define CONFIG_SYS_HUSH_PARSER
#endif

#define CONFIG_CMD_RUN		/* run command in env variable	*/
#define CONFIG_CMD_SAVEENV	/* saveenv			*/

#define CONFIG_BOOTDELAY               1

#define CONFIG_LOADADDR                0x12000000
#define CONFIG_SYS_TEXT_BASE           0x17800000

#define CONFIG_PREBOOT

#define CONFIG_EXTRA_ENV_SETTINGS \
	"baurate=115200\0" \
	"fdt_file=" __stringify(CONFIG_DEFAULT_FDT_FILE) "\0" \
	"fdt_addr=0x18000000\0" \
	"fdt_high=0xffffffff\0" \
	"boot_fdt=try\0" \
	"bootsel=1\0" \
	"debug=quiet\0" \
	"rw=ro\0" \
	"cpus=maxcpus=1\0" \
	"mem=mem=768M\0" \
	"bootstd=setenv bootargs panic=5 console=" __stringify(CONFIG_CONSOLE_DEV) ",${baudrate} ${debug} ${rw} ${cpus} ${mem}\0" \
	"bootpart=if test \"m$mtdparts\" = \"m\"; then mtdparts default; else true; fi\0" \
	"bootenv_nand=setenv bootargs ${bootargs} ${mtdparts} root=ubi0:rootfs rootfstype=ubifs rootflags=no_chk_data_crc ubi.mtd=linux${bootsel}\0" \
	"select_os=ubi part linux${bootsel}\0" \
	"get_kernel_nand=ubi read ${loadaddr} kernel; ubi read ${fdt_addr} dtb\0" \
	"get_fdt=ubi read ${fdt_addr} fdt\0" \
	"bootcmd_nand=run select_os get_kernel_nand bootstd bootenv_nand linux\0" \
	"bootenv_mmc=setenv bootargs ${bootargs} $mtdparts root=" __stringify(CONFIG_MMCROOT) " rootwait ip=off\0" \
	"bootcmd_mmc=run get_kernel_mmc bootpart bootstd bootenv_mmc linux\0" \
	"linux=bootm ${loadaddr} - ${fdt_addr}\0" \
	"get_kernel_mmc=mmc dev 0; " \
		"ext2load mmc0 0:1 ${loadaddr} /boot/uImage;" \
		"ext2load mmc0 0:1 ${fdt_addr} /boot/${fdt_file}\0" \
	"get_flowctrl_mmc=mmc dev 0;  " \
		"ext2load mmc0 0:1 40000000 /boot/flowctrl.bin\0" \
	"boot_flowctrl=cpu 3 release 40000000 0x40000298\0" \
	"flowctrl=run get_flowctrl_mmc boot_flowctrl\0" \
	"operating_mode=bringup\0"

#define MTDIDS_DEFAULT		"nand0=gpmi-nand"
#define MTDPARTS_DEFAULT	"mtdparts=gpmi-nand:2m(u-boot),128k(env1),128k(env2),150m(linux1),150m(linux2),-(user)"

#define CONFIG_BOOTCOMMAND	"run bootcmd_mmc"

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "
#define CONFIG_SYS_PROMPT              "U-Boot > "
#undef  CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE              256

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS             16
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END         0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

#define CONFIG_SYS_LOAD_ADDR           CONFIG_LOADADDR
#define CONFIG_SYS_HZ                  1000

#undef  CONFIG_CMDLINE_EDITING
#define CONFIG_STACKSIZE               (128 * 1024)

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS           1
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/*
 * GPMI Nand Configs
 */
#ifdef CONFIG_CMD_NAND
#	define CONFIG_NAND_MXS
#	define CONFIG_SYS_MAX_NAND_DEVICE	1
#	define CONFIG_SYS_NAND_BASE		0x40000000
#	define CONFIG_SYS_NAND_ONFI_DETECTION

#	define CONFIG_APBH_DMA
//#	define CONFIG_APBH_DMA_BURST
//#	define CONFIG_APBH_DMA_BURST8
#endif

/* FLASH and environment organization */

#ifdef CONFIG_CMD_NAND
#	define CONFIG_ENV_IS_IN_NAND
#	define CONFIG_ENV_OFFSET               0x200000
#	define CONFIG_ENV_SECT_SIZE            SZ_128K
#	define CONFIG_ENV_SIZE                 CONFIG_ENV_SECT_SIZE
#	define CONFIG_ENV_OFFSET_REDUND        (CONFIG_ENV_OFFSET + CONFIG_ENV_SIZE)
#	define CONFIG_ENV_SIZE_REDUND          CONFIG_ENV_SIZE
#else
#	define CONFIG_ENV_SIZE			(8 * 1024)
#	define CONFIG_ENV_IS_IN_MMC
#endif

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(512 * 1024)
#endif

#define CONFIG_OF_LIBFDT

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

/*
 * I2C configs
 */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000
//#define CONFIG_SYS_MXC_I2C2_SPEED	100000
//#define CONFIG_SYS_MXC_I2C2_SLAVE	0x8

#endif                         /* __MX6QMARYANN_COMMON_CONFIG_H */
