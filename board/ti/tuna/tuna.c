/*
 * (C) Copyright 2012
 * Alexander Tarasikov <alexander.tarasikov@gmail.com>
 *
 * based on panda board which is
 * (C) Copyright 2010
 * Texas Instruments Incorporated, <www.ti.com>
 * Steve Sakoman  <steve@sakoman.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <common.h>
#include <mmc.h>
#include <i2c.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/clocks.h>
#include <asm/arch/gpio.h>
#include <asm/gpio.h>

#include "tuna_mux_data.h"

#ifdef CONFIG_USB_EHCI
#include <usb.h>
#include <asm/arch/ehci.h>
#include <asm/ehci-omap.h>
#endif

#define PANDA_ULPI_PHY_TYPE_GPIO       182

DECLARE_GLOBAL_DATA_PTR;

const struct omap_sysinfo sysinfo = {
	"Board: OMAP4 Panda\n"
};

struct omap4_scrm_regs *const scrm = (struct omap4_scrm_regs *)0x4a30a000;

static void tuna_clear_i2c4(void) {
	unsigned r;

	r = readl(CONTROL_PADCONF_CORE + 0x604);
	r |= (1 << 28);
	writel(r, CONTROL_PADCONF_CORE + 0x604);
}

/******************************************************************************
 * Revision detection
 *****************************************************************************/

#define TUNA_REV_MASK		0xf
#define TUNA_REV_03			0x3
#define TUNA_REV_SAMPLE_4	0x3

#define TUNA_TYPE_TORO		0x10
#define TUNA_TYPE_MAGURO	0x00
#define TUNA_TYPE_MASK		0x10

static int tuna_hw_rev = -1;

static const char const *omap4_tuna_hw_name_maguro[] = {
	[0x00] = "Toro Lunchbox #1",
	[0x01] = "Maguro 1st Sample",
	[0x02] = "Maguro 2nd Sample",
	[0x03] = "Maguro 4th Sample",
	[0x05] = "Maguro 5th Sample",
	[0x07] = "Maguro 6th Sample",
	[0x08] = "Maguro 7th Sample",
	[0x09] = "Maguro 8th Sample",
};

static const char const *omap4_tuna_hw_name_toro[] = {
	[0x00] = "Toro Lunchbox #2",
	[0x01] = "Toro 1st Sample",
	[0x02] = "Toro 2nd Sample",
	[0x03] = "Toro 4th Sample",
	[0x05] = "Toro 5th Sample",
	[0x06] = "Toro 8th Sample",
	[0x08] = "Toro 8th Sample",
	[0x09] = "Toro 8-1th Sample",
	[0x0e] = "Toro Plus 1st Sample",
};

static int omap4_tuna_get_revision(void)
{
	return tuna_hw_rev & TUNA_REV_MASK;
}

static int omap4_tuna_get_type(void)
{
	return tuna_hw_rev & TUNA_TYPE_MASK;
}

static const char *omap4_tuna_hw_rev_name(void) {
	const char *ret;
	const char **names;
	int num;
	int rev;

	if (omap4_tuna_get_type() == TUNA_TYPE_MAGURO) {
		names = omap4_tuna_hw_name_maguro;
		num = ARRAY_SIZE(omap4_tuna_hw_name_maguro);
		ret = "Maguro unknown";
	} else {
		names = omap4_tuna_hw_name_toro;
		num = ARRAY_SIZE(omap4_tuna_hw_name_toro);
		ret = "Toro unknown";
	}

	rev = omap4_tuna_get_revision();
	if (rev >= num || !names[rev])
		return ret;

	return names[rev];
}

static void gnex_get_revision(void) {
	size_t i;
	int revision = 0;
	unsigned gpios[] = {
		76,
		75,
		74,
		73,
		170,
	};
	unsigned r;
	
	//disable usb HSIC pullup
	r = readl(CONTROL_PADCONF_CORE + 0x5c4);
	r &= ~(3 << 14);
	writel(r, CONTROL_PADCONF_CORE + 0x5c4);
	
	for (i = 0; i < ARRAY_SIZE(gpios); i++) {
		gpio_direction_input(gpios[i]);
		revision |= (gpio_get_value(gpios[i]) << i);
	}
	tuna_hw_rev = revision;
}

int do_tuna_print_revision(cmd_tbl_t *cmdtp, int flag,
	int argc, char * const argv[])
{
	gnex_get_revision();
	const char *rev_name = omap4_tuna_hw_rev_name();
	if (!rev_name) {
		return -1;
	}
	printf("Tuna revision %d: %s\n", tuna_hw_rev, rev_name);
	return 0;
}

U_BOOT_CMD(tuna_print_revision, CONFIG_SYS_MAXARGS, 1, do_tuna_print_revision,
	"Print Tuna (Galaxy Nexus) revision\n",
	"tuna_print_revision\n"
);

/******************************************************************************
 * Serial ATAG
 *****************************************************************************/
void get_board_serial(struct tag_serialnr *serialnr) {
	serialnr->low = 0;
	serialnr->high = 0;
}

/******************************************************************************
 * Color led control
 *****************************************************************************/
static void tuna_set_led(int color) {
	u8 val, reg;

	tuna_clear_i2c4();
	i2c_set_bus_num(3);

	//reset
	reg = 0;
	val = 1;
	i2c_write(0x30, reg, 1, &val, 1);

	//led select
	reg = 1;
	val = color & 7;
	i2c_write(0x30, reg, 1, &val, 1);

	//led enable
	reg = 2;
	val = color & 7;
	i2c_write(0x30, reg, 1, &val, 1);
	
	reg = 3;
	val = color & 1 ? 0xff : 0;
	i2c_write(0x30, reg, 1, &val, 1);

	reg = 4;
	val = color & 2 ? 0xff : 0;
	i2c_write(0x30, reg, 1, &val, 1);

	reg = 5;
	val = color & 4 ? 0xff : 0;
	i2c_write(0x30, reg, 1, &val, 1);

	i2c_set_bus_num(0);
}

int do_tuna_set_led(cmd_tbl_t *cmdtp, int flag,
	int argc, char * const argv[])
{
	if (argc < 2) {
		return -1;
	}
	tuna_set_led(argv[1][0] - '0');
	return 0;
}

U_BOOT_CMD(tuna_set_led, CONFIG_SYS_MAXARGS, 1, do_tuna_set_led,
	"Set Tuna (Galaxy Nexus) color LEDs\n",
	"tuna_set_led\n"
);

/******************************************************************************
 * Getting boot reason
 *****************************************************************************/
//These are the values the kernel sets the bootflag register to on reboot */
#define REBOOT_FLAG_RECOVERY	0x52564352
#define REBOOT_FLAG_FASTBOOT	0x54534146
#define REBOOT_FLAG_NORMAL	0x4D524F4E
#define REBOOT_FLAG_POWER_OFF	0x46464F50

/* this is the register to store boot mode which is preserved over a reboot */
#define SAMSUNG_BOOTFLAG_ADDR 0x4a326ff4

enum bootmode {
	BOOTMODE_NORMAL,
	BOOTMODE_RECOVERY,
	BOOTMODE_CUSTOM,
	BOOTMODE_USBDEBUG,
	BOOTMODE_POWEROFF,
	BOOTMODE_UNKNOWN,
};

static enum bootmode tuna_bootmode = BOOTMODE_UNKNOWN;

static void tuna_check_bootflag(void) {
	unsigned flag;

	flag = readl(SAMSUNG_BOOTFLAG_ADDR);

	switch (flag) {
		case REBOOT_FLAG_RECOVERY:
			tuna_bootmode = BOOTMODE_RECOVERY;
			break;
		case REBOOT_FLAG_NORMAL:
			tuna_bootmode = BOOTMODE_NORMAL;
			break;
		case REBOOT_FLAG_FASTBOOT:
			tuna_bootmode = BOOTMODE_USBDEBUG;
			break;
		case REBOOT_FLAG_POWER_OFF:	
			tuna_bootmode = BOOTMODE_POWEROFF;
			break;
	}
}

int do_tuna_get_bootmode(cmd_tbl_t *cmdtp, int flag,
	int argc, char * const argv[])
{
	gpio_direction_input(30);
	int volup = !gpio_get_value(30);
	
	gpio_direction_input(8);
	int voldown = !gpio_get_value(8);
	
	tuna_bootmode = (voldown << 1) | volup;

	if (tuna_bootmode == BOOTMODE_NORMAL) {
		tuna_check_bootflag();
	}

#if defined(CONFIG_TWL6030_POWER)
	if (tuna_bootmode == BOOTMODE_POWEROFF) {
		twl6030_system_shutdown();
	}
#endif

	setenv("tuna_bootmode_val", simple_itoa(tuna_bootmode));
	return 0;
}

U_BOOT_CMD(tuna_get_bootmode, CONFIG_SYS_MAXARGS, 1, do_tuna_get_bootmode,
	"Get Tuna (Galaxy Nexus) boot mode\n"
	"Bit 0 -> recovery\n"
	"Bit 1 -> custom kernel",
	"tuna_get_bootmode\n"
);

/******************************************************************************
 * Framebuffer
 *****************************************************************************/
#ifdef CONFIG_VIDEO
#include <video_fb.h>

GraphicDevice gdev;

#define TUNA_FB 0xbea70000
#define TUNA_XRES 720
#define TUNA_YRES 1280

void *video_hw_init(void) {
	memset((void*)TUNA_FB, 0x00, TUNA_XRES * TUNA_YRES * 4);
	gdev.frameAdrs = TUNA_FB;
	gdev.winSizeX = TUNA_XRES;
	gdev.winSizeY = TUNA_YRES;
	gdev.gdfBytesPP = 4;
	gdev.gdfIndex = GDF_24BIT_888RGB;
	return &gdev;
}
#endif

/**
 * @brief board_init
 *
 * @return 0
 */

int board_init(void)
{
	//since we can boot either from uboot, Samsung SBL or natively,
	//we want to always reinit mux regardless of what
	//the code in arch/arm/cpu/armv7/omap-common is thinking
	set_muxconf_regs_essential();
	gpmc_init();

	gd->bd->bi_arch_number = MACH_TYPE_TUNA;
	gd->bd->bi_boot_params = (0x80000000 + 0x100); /* boot param addr */
	gd->ram_size = 0x40000000;
	
	gd->bd->bi_dram[0].start = 0x80000000;
	gd->bd->bi_dram[0].size =  0x40000000;
	
	//indicate we're alive
	tuna_set_led(5);

	tuna_check_bootflag();

	return 0;
}

int board_eth_init(bd_t *bis)
{
	return 0;
}

/**
 * @brief misc_init_r - Configure Panda board specific configurations
 * such as power configurations, ethernet initialization as phase2 of
 * boot sequence
 *
 * @return 0
 */
int misc_init_r(void)
{
	int phy_type;
	u32 auxclk, altclksrc;

	/* EHCI is not supported on ES1.0 */
	if (omap_revision() == OMAP4430_ES1_0)
		return 0;

	gpio_direction_input(PANDA_ULPI_PHY_TYPE_GPIO);
	phy_type = gpio_get_value(PANDA_ULPI_PHY_TYPE_GPIO);

	if (phy_type == 1) {
		/* ULPI PHY supplied by auxclk3 derived from sys_clk */
		debug("ULPI PHY supplied by auxclk3\n");

		auxclk = readl(&scrm->auxclk3);
		/* Select sys_clk */
		auxclk &= ~AUXCLK_SRCSELECT_MASK;
		auxclk |=  AUXCLK_SRCSELECT_SYS_CLK << AUXCLK_SRCSELECT_SHIFT;
		/* Set the divisor to 2 */
		auxclk &= ~AUXCLK_CLKDIV_MASK;
		auxclk |= AUXCLK_CLKDIV_2 << AUXCLK_CLKDIV_SHIFT;
		/* Request auxilary clock #3 */
		auxclk |= AUXCLK_ENABLE_MASK;

		writel(auxclk, &scrm->auxclk3);
       } else {
		/* ULPI PHY supplied by auxclk1 derived from PER dpll */
		debug("ULPI PHY supplied by auxclk1\n");

		auxclk = readl(&scrm->auxclk1);
		/* Select per DPLL */
		auxclk &= ~AUXCLK_SRCSELECT_MASK;
		auxclk |=  AUXCLK_SRCSELECT_PER_DPLL << AUXCLK_SRCSELECT_SHIFT;
		/* Set the divisor to 16 */
		auxclk &= ~AUXCLK_CLKDIV_MASK;
		auxclk |= AUXCLK_CLKDIV_16 << AUXCLK_CLKDIV_SHIFT;
		/* Request auxilary clock #3 */
		auxclk |= AUXCLK_ENABLE_MASK;

		writel(auxclk, &scrm->auxclk1);
	}

	altclksrc = readl(&scrm->altclksrc);

	/* Activate alternate system clock supplier */
	altclksrc &= ~ALTCLKSRC_MODE_MASK;
	altclksrc |= ALTCLKSRC_MODE_ACTIVE;

	/* enable clocks */
	altclksrc |= ALTCLKSRC_ENABLE_INT_MASK | ALTCLKSRC_ENABLE_EXT_MASK;

	writel(altclksrc, &scrm->altclksrc);
	return 0;
}

void set_muxconf_regs_essential(void)
{
	do_set_mux(CONTROL_PADCONF_CORE, core_padconf_array_essential,
		   sizeof(core_padconf_array_essential) /
		   sizeof(struct pad_conf_entry));

	do_set_mux(CONTROL_PADCONF_WKUP, wkup_padconf_array_essential,
		   sizeof(wkup_padconf_array_essential) /
		   sizeof(struct pad_conf_entry));

	if (omap_revision() >= OMAP4460_ES1_0)
		do_set_mux(CONTROL_PADCONF_WKUP,
				 wkup_padconf_array_essential_4460,
				 sizeof(wkup_padconf_array_essential_4460) /
				 sizeof(struct pad_conf_entry));
}

void set_muxconf_regs_non_essential(void)
{
	do_set_mux(CONTROL_PADCONF_CORE, core_padconf_array_non_essential,
		   sizeof(core_padconf_array_non_essential) /
		   sizeof(struct pad_conf_entry));
	
	if (omap_revision() < OMAP4460_ES1_0)
		do_set_mux(CONTROL_PADCONF_CORE,
				core_padconf_array_non_essential_4430,
				sizeof(core_padconf_array_non_essential_4430) /
				sizeof(struct pad_conf_entry));
	else
		do_set_mux(CONTROL_PADCONF_CORE,
				core_padconf_array_non_essential_4460,
				sizeof(core_padconf_array_non_essential_4460) /
				sizeof(struct pad_conf_entry));

	do_set_mux(CONTROL_PADCONF_WKUP, wkup_padconf_array_non_essential,
		   sizeof(wkup_padconf_array_non_essential) /
		   sizeof(struct pad_conf_entry));

	if (omap_revision() < OMAP4460_ES1_0)
		do_set_mux(CONTROL_PADCONF_WKUP,
				wkup_padconf_array_non_essential_4430,
				sizeof(wkup_padconf_array_non_essential_4430) /
				sizeof(struct pad_conf_entry));
}


#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_GENERIC_MMC)
int board_mmc_init(bd_t *bis)
{
	//unlock TWL6030 PMIC for writing
	gpio_direction_output(6, 1);
	gpio_set_value(6, 1);

	//enable EMMC power
	gpio_direction_output(158, 1);
	gpio_set_value(158, 1);

	i2c_set_bus_num(0);
	omap_mmc_init(0, 0, 0);
	return 0;
}
#endif

#ifdef CONFIG_USB_EHCI

static struct omap_usbhs_board_data usbhs_bdata = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
};

int ehci_hcd_init(void)
{
	int ret;
	unsigned int utmi_clk;

	/* Now we can enable our port clocks */
	utmi_clk = readl((void *)CM_L3INIT_HSUSBHOST_CLKCTRL);
	utmi_clk |= HSUSBHOST_CLKCTRL_CLKSEL_UTMI_P1_MASK;
	sr32((void *)CM_L3INIT_HSUSBHOST_CLKCTRL, 0, 32, utmi_clk);

	ret = omap_ehci_hcd_init(&usbhs_bdata);
	if (ret < 0)
		return ret;

	return 0;
}

int ehci_hcd_stop(void)
{
	return omap_ehci_hcd_stop();
}
#endif

/*
 * get_board_rev() - get board revision
 */
u32 get_board_rev(void)
{
	gnex_get_revision();
	return omap4_tuna_get_revision();
}
