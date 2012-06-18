/*
 * (C) Copyright 2012
 * Alexander Tarasikov <alexander.tarasikov@gmail.com>
 *
 * based on panda code which is
 * (C) Copyright 2010
 * Texas Instruments Incorporated.
 * Steve Sakoman  <steve@sakoman.com>
 *
 * Configuration settings for the TI OMAP4 Panda board.
 * See omap4_common.h for OMAP4 common part
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_TUNA_H
#define __CONFIG_TUNA_H

/*
 * High Level Configuration Options
 */
#define CONFIG_TUNA	/* working with Panda */

/*
 * High Level Configuration Options
 */
#define CONFIG_ARMV7		1	/* This is an ARM V7 CPU core */
#define CONFIG_OMAP		1	/* in a TI OMAP core */
#define CONFIG_OMAP44XX		1	/* which is a 44XX */
#define CONFIG_OMAP4430		1	/* which is in a 4430 */

#define TUNA_SPL_BUILD

//#define CONFIG_SKIP_LOWLEVEL_INIT 1
//#define CONFIG_SYS_DCACHE_OFF 1
//#define CONFIG_SYS_ICACHE_OFF 1
//#define CONFIG_SYS_L2CACHE_OFF 1

/* Get CPU defs */
#include <asm/arch/cpu.h>
#include <asm/arch/omap.h>

#define CONFIG_MACH_TYPE		MACH_TYPE_TUNA

/* Display CPU and Board Info */
#define CONFIG_DISPLAY_CPUINFO		1
#define CONFIG_DISPLAY_BOARDINFO	1

/* Clock Defines */
#define V_OSCK			38400000	/* Clock output from T2 */
#define V_SCLK                   V_OSCK

#undef CONFIG_USE_IRQ				/* no support for IRQs */
#define CONFIG_MISC_INIT_R

#define CONFIG_CMDLINE_TAG		1	/* enable passing of ATAGs */
#define CONFIG_SETUP_MEMORY_TAGS	1
#define CONFIG_INITRD_TAG		1
#define CONFIG_REVISION_TAG		1
#define CONFIG_SERIAL_TAG	1

/*
 * Size of malloc() pool
 * Total Size Environment - 128k
 * Malloc - add 256k
 */
#define CONFIG_ENV_SIZE			(128 << 10)
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + (256 << 10))
/* Vector Base */
#define CONFIG_SYS_CA9_VECTOR_BASE	SRAM_ROM_VECT_BASE

/*
 * Hardware drivers
 */

/*
 * serial port - NS16550 compatible
 */
#define V_NS16550_CLK			48000000

#define CONFIG_SYS_NS16550
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	(-4)
#define CONFIG_SYS_NS16550_CLK		V_NS16550_CLK
#define CONFIG_CONS_INDEX		3
#define CONFIG_SYS_NS16550_COM3		UART3_BASE

#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{4800, 9600, 19200, 38400, 57600,\
					115200}
/* I2C  */
#define CONFIG_HARD_I2C			1
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_SYS_I2C_SLAVE		1
#define CONFIG_SYS_I2C_BUS		0
#define CONFIG_SYS_I2C_BUS_SELECT	1
#define CONFIG_DRIVER_OMAP34XX_I2C	1
#define CONFIG_I2C_MULTI_BUS		1

/* TWL6030 */
#define CONFIG_TWL6030_POWER		1

/* MMC */
#define CONFIG_GENERIC_MMC		1
#define CONFIG_MMC			1
#define CONFIG_OMAP_HSMMC		1
#define CONFIG_EFI_PARTITION		1
#define OMAP4_MMC_NO_VMODE 1

/* USB */
#define CONFIG_MUSB_UDC			1
#define CONFIG_USB_OMAP3		1
#define CONFIG_USB_ULPI
#define CONFIG_USB_ULPI_VIEWPORT_OMAP

/* USB Product */
#define CONFIG_USBD_MANUFACTURER "Samsung"
#define CONFIG_USBD_PRODUCT_NAME "Galaxy Nexus"

/* USB device configuration */
#define CONFIG_USB_DEVICE		1
#define CONFIG_USB_TTY			1
#define CONFIG_EXTRA_ENV_USBTTY "usbtty=cdc_acm\0"

//#define CONFIG_SYS_CONSOLE_IS_IN_ENV	1

/* Flash */
#define CONFIG_SYS_NO_FLASH	1

/* clocks */
#if 0
#define CONFIG_SYS_CLOCKS_ENABLE_ALL
#endif

/* commands to include */
#include <config_cmd_default.h>

/* Enabled commands */
#define CONFIG_CMD_EXT2		/* EXT2 Support                 */
#define CONFIG_CMD_FAT		/* FAT support                  */
#define CONFIG_CMD_I2C		/* I2C serial bus support	*/
#define CONFIG_CMD_MMC		/* MMC support                  */
#define CONFIG_CMD_BOOTZ

/* Disabled commands */
#undef CONFIG_CMD_NET
#undef CONFIG_CMD_NFS
#undef CONFIG_CMD_FPGA		/* FPGA configuration Support   */
#undef CONFIG_CMD_IMLS		/* List all found images        */
#undef CONFIG_CMD_IMI
#undef CONFIG_CMD_FLASH

/*
 * Environment setup
 */
#define CONFIG_BOOTDELAY	0
#define CONFIG_ZERO_BOOTDELAY_CHECK
#define CONFIG_ENV_OVERWRITE

#define CONFIG_ANDROID_BOOT_IMAGE

#define ANDROID_CMDLINE " mem=1G vmalloc=768M" \
	" omap_wdt.timer_margin=30" \
	" mms_ts.panel_id=18" \
	" no_console_suspend" \
	" console=ttyFIQ0 "

/* mmc partitions
 * 7 -> boot 0x14000 0x4000
 * 8 -> recovery 0x18000 0x6000
 * 10 -> system
 * 12 -> userdata
 */

#define CONFIG_EXTRA_ENV_SETTINGS \
	\
	"loadaddr=0x82000000\0" \
	"usbtty=cdc_acm\0" \
	"kernel_name=/boot/vmlinux.uimg\0" \
	"script_img/boot/boot.scr.uimg\0" \
	\
	"load_boot_script=if fatload ${devtype} ${devnum}:${script_part} " \
			"${loadaddr} ${script_img}; then " \
			"source ${loadaddr}; " \
		"elif ext2load ${devtype} ${devnum}:${script_part} " \
				"${loadaddr} ${script_img}; then " \
			"source ${loadaddr}; " \
		"fi\0" \
	\
	"custom_boot=setenv bootargs "\
			"${dev_extras} root=/dev/${devname}${rootpart} rootwait ro ;"\
		"echo Load Address:${loadaddr};" \
		"echo Cmdline:${bootargs}; " \
		"if fatload ${devtype} ${devnum}:${kernel_part} " \
			"${loadaddr} ${kernel_name}; then " \
			"bootm ${loadaddr}; " \
		"elif ext2load ${devtype} ${devnum}:${kernel_part} " \
		            "${loadaddr} ${kernel_name}; then " \
			"bootm ${loadaddr};" \
		"fi\0" \
	\
	"boot_custom_emmc=echo Booting custom image; " \
		"tuna_set_led 4; " \
		"setenv script_part 0xc; " \
		"setenv kernel_part 0xc; " \
		"setenv rootpart 0xc; " \
		"setenv devnum 0; " \
		"setenv devtype mmc; " \
		"run load_boot_script; " \
		"run custom_boot\0" \
	\
	"boot_recovery=echo Booting RECOVERY; " \
		"tuna_set_led 2; " \
		"setenv bootargs " ANDROID_CMDLINE " ; " \
		"mmc dev 0; " \
		"mmc read ${loadaddr} 0x18000 0x6000; "\
		"echo Command line: ${bootargs}; " \
		"bootm ${loadaddr}\0" \
	\
	"boot_android=echo Booting ANDROID; " \
		"tuna_set_led 1; " \
		"setenv bootargs " ANDROID_CMDLINE " ; " \
		"mmc dev 0; " \
		"mmc read ${loadaddr} 0x14000 0x4000; "\
		"echo Command line: ${bootargs}; " \
		"bootm ${loadaddr}\0" \
	\
	"go_usbtty=setenv stdin usbtty; " \
		"setenv stdout usbtty; " \
		"setenv stderr usbtty; " \
		"tuna_set_led 3\0" \
	\
	"tuna_boot=mmc rescan; " \
		"mmc dev 0; " \
		"mmc part; " \
		"tuna_get_bootmode; " \
		"if test $tuna_bootmode_val -eq 0; then " \
			"echo Regular boot; " \
			"run boot_android; " \
		"elif test $tuna_bootmode_val -eq 1; then " \
			"echo Recovery boot; " \
			"run boot_recovery; " \
		"elif test $tuna_bootmode_val -eq 2; then " \
			"echo Custom boot from userdata; " \
			"run boot_custom_emmc; " \
		"elif test $tuna_bootmode_val -eq 3; then " \
			"echo USB TTY mode; " \
			"run go_usbtty; " \
			"exit 0; " \
		"fi; " \
		"tuna_set_led 7; " \
		"echo Failed to boot\0"

#define CONFIG_BOOTCOMMAND \
	"echo Booting up ;" \
	"tuna_get_bootmode; "\
	"echo [Tuna] bootmode $tuna_bootmode_val; " \
	"tuna_print_revision ; " \
	"run tuna_boot ;"

/*
 * Miscellaneous configurable options
 */
#define CONFIG_AUTO_COMPLETE 1
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER	/* use "hush" command parser */
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#define CONFIG_SYS_CBSIZE		512
/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
					sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS		16
/* Boot Argument Buffer Size */
#define CONFIG_SYS_BARGSIZE		(CONFIG_SYS_CBSIZE)

/*
 * memtest setup
 */
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + (32 << 20))

/* Default load address */
#define CONFIG_SYS_LOAD_ADDR		0x82000000

/* Use General purpose timer 1 */
#define CONFIG_SYS_TIMERBASE		GPT2_BASE
#define CONFIG_SYS_PTV			2	/* Divisor: 2^(PTV+1) => 8 */
#define CONFIG_SYS_HZ			1000

/*
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE	(128 << 10)	/* Regular stack */
#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ	(4 << 10)	/* IRQ stack */
#define CONFIG_STACKSIZE_FIQ	(4 << 10)	/* FIQ stack */
#endif

/*
 * SDRAM Memory Map
 * Even though we use two CS all the memory
 * is mapped to one contiguous block
 */
#define CONFIG_NR_DRAM_BANKS	1

#define CONFIG_SYS_SDRAM_BASE		0x80000000

#define CONFIG_SYS_INIT_RAM_ADDR	0x4030D800
#define CONFIG_SYS_INIT_RAM_SIZE	0x800
#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_INIT_RAM_ADDR + \
					 CONFIG_SYS_INIT_RAM_SIZE - \
					 GENERATED_GBL_DATA_SIZE)

#ifndef CONFIG_SYS_L2CACHE_OFF
#define CONFIG_SYS_L2_PL310		1
#define CONFIG_SYS_PL310_BASE	0x48242000
#endif
#define CONFIG_SYS_CACHELINE_SIZE	32

/* Defines for SDRAM init */
#define CONFIG_SYS_EMIF_PRECALCULATED_TIMING_REGS

#ifndef CONFIG_SYS_EMIF_PRECALCULATED_TIMING_REGS
#define CONFIG_SYS_AUTOMATIC_SDRAM_DETECTION
#define CONFIG_SYS_DEFAULT_LPDDR2_TIMINGS
#endif

#ifndef TUNA_SPL_BUILD
	#define CONFIG_VIDEO
	#define CONFIG_CFB_CONSOLE
	#define CONFIG_VGA_AS_SINGLE_DEVICE
	#define CONFIG_STD_DEVICES_SETTINGS "stdin=vga\0" \
									"stdout=vga\0" \
									"stderr=vga\0"
#else
	#define CONFIG_STD_DEVICES_SETTINGS "stdin=usbtty\0" \
									"stdout=usbtty\0" \
									"stderr=usbtty\0"
#endif

/*
 * 64 bytes before this address should be set aside for u-boot.img's
 * header. That is 80E7FFC0--0x80E80000 should not be used for any
 * other needs.
 */

/*
 * Samsung xloader loads SBL at 0xa0208000
 *
 */

#ifndef TUNA_SPL_BUILD
	#define CONFIG_SYS_TEXT_BASE		0x81808000
#else
	#define CONFIG_SYS_TEXT_BASE		0xa0208000
#endif

#define CONFIG_SYS_ENABLE_PADS_ALL
#define CONFIG_UBOOT_ENABLE_PADS_ALL
#define CONFIG_SYS_THUMB_BUILD

/* GPIO */
#define CONFIG_CMD_GPIO

/* ENV related config options */
#define CONFIG_ENV_IS_NOWHERE
#define CONFIG_SYS_PROMPT		"Galaxy Nexus # "

#endif /* __CONFIG_TUNA_H */
