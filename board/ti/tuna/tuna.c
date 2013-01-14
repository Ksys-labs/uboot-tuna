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

#include <asm/omap_musb.h>
#include <asm/errno.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/musb.h>

#ifdef CONFIG_USB_EHCI
#include <usb.h>
#include <asm/arch/ehci.h>
#include <asm/ehci-omap.h>
#endif

#ifdef CONFIG_TWL6030_POWER
#include <twl6030.h>
#endif

#define TUNA_GPIO_USB3333_RESETB 159

DECLARE_GLOBAL_DATA_PTR;

const struct omap_sysinfo sysinfo = {
	"Board: OMAP4 Tuna\n"
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

static int tuna_hw_rev = 0;

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
	static unsigned gpios[] = {
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
		revision |= (!!gpio_get_value(gpios[i])) << i;
	}

	tuna_hw_rev = revision;
	printf("Tuna revision %d: %s\n",
		tuna_hw_rev, omap4_tuna_hw_rev_name());
}

int do_tuna_print_revision(cmd_tbl_t *cmdtp, int flag,
	int argc, char * const argv[])
{
	gnex_get_revision();
	const char *rev_name = omap4_tuna_hw_rev_name();
	if (!rev_name) {
		return -1;
	}
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
enum {
	RGB_LED_ADDR = 0x30,
	RGB_REG_RESET = 0,
	RGB_REG_RESET_VAL = 1,

	RGB_REG_SELECT = 1,
	RGB_REG_ENABLE = 2,

	RGB_REG_RED = 3,
	RGB_REG_GREEN = 4,
	RGB_REG_BLUE = 5,

	RGB_LED_RED = 1,
	RGB_LED_GREEN = 2,
	RGB_LED_BLUE = 4,
	RGB_LED_MASK = 7,
};

static void an30259_set_led(unsigned chip_addr, int color) {
	u8 val, reg;

	//reset
	reg = RGB_REG_RESET;
	val = RGB_REG_RESET_VAL;
	i2c_write(chip_addr, reg, 1, &val, 1);

	//led select
	reg = RGB_REG_SELECT;
	val = color & RGB_LED_MASK;
	i2c_write(chip_addr, reg, 1, &val, 1);

	//led enable
	reg = RGB_REG_ENABLE;
	val = color & RGB_LED_MASK;
	i2c_write(chip_addr, reg, 1, &val, 1);
	
	reg = RGB_REG_RED;
	val = color & RGB_LED_RED ? 0xff : 0;
	i2c_write(chip_addr, reg, 1, &val, 1);

	reg = RGB_REG_GREEN;
	val = color & RGB_LED_GREEN ? 0xff : 0;
	i2c_write(chip_addr, reg, 1, &val, 1);

	reg = RGB_REG_BLUE;
	val = color & RGB_LED_BLUE ? 0xff : 0;
	i2c_write(chip_addr, reg, 1, &val, 1);
}

#define TUNA_AN30259_ADDR 0x30

static void tuna_set_led(int color) {
	tuna_clear_i2c4();
	i2c_set_bus_num(3);
	an30259_set_led(TUNA_AN30259_ADDR, color);
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

	//reset bootmode flag to avoid getting stuck in the boot loop
	if (tuna_bootmode == BOOTMODE_RECOVERY || tuna_bootmode == BOOTMODE_USBDEBUG) {
		writel(REBOOT_FLAG_NORMAL, SAMSUNG_BOOTFLAG_ADDR);
	}

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
 * FSA9480 USB Detection (XXX: factor out to a separate driver)
 *****************************************************************************/
typedef enum {
	FSA9480_UNKNOWN,
	FSA9480_USB,
	FSA9480_USB_HOST,
	FSA9480_CHARGER,
	FSA9480_JIG,
	FSA9480_UART,
	FSA9480_AV_365K, //what is that?
	FSA9480_AV_365K_CHARGER,
} fsa9480_dev_t;

#define FSA9480_REG_DEVID		0x01
#define FSA9480_REG_CTRL		0x02
#define FSA9480_REG_INT1		0x03
#define FSA9480_REG_INT2		0x04
#define FSA9480_REG_INT1_MASK		0x05
#define FSA9480_REG_INT2_MASK		0x06
#define FSA9480_REG_ADC			0x07
#define FSA9480_REG_TIMING1		0x08
#define FSA9480_REG_TIMING2		0x09
#define FSA9480_REG_DEV_T1		0x0a
#define FSA9480_REG_DEV_T2		0x0b
#define FSA9480_REG_BTN1		0x0c
#define FSA9480_REG_BTN2		0x0d
#define FSA9480_REG_CK			0x0e
#define FSA9480_REG_CK_INT1		0x0f
#define FSA9480_REG_CK_INT2		0x10
#define FSA9480_REG_CK_INTMASK1		0x11
#define FSA9480_REG_CK_INTMASK2		0x12
#define FSA9480_REG_MANSW1		0x13
#define FSA9480_REG_MANSW2		0x14
#define FSA9480_REG_ANALOG_TEST		0x15
#define FSA9480_REG_SCAN_TEST		0x16
#define FSA9480_REG_DAC_OVERRIDE_1	0x17
#define FSA9480_REG_DAC_OVERRIDE_2	0x18
#define FSA9480_REG_VIDEO_DETECT	0x19
#define FSA9480_REG_CK_PULSE_WIDTH	0x1A
#define FSA9480_REG_MANOVERRIDE1	0x1B
#define FSA9480_REG_STATUS1		0x1C
#define FSA9480_REG_STATUS2		0x1D
#define FSA9480_REG_FUSE1		0x1E

/* Control */
#define CON_SWITCH_OPEN		(1 << 4)
#define CON_RAW_DATA		(1 << 3)
#define CON_MANUAL_SW		(1 << 2)
#define CON_WAIT		(1 << 1)
#define CON_INT_MASK		(1 << 0)
#define CON_MASK		(CON_SWITCH_OPEN | CON_RAW_DATA | \
				CON_MANUAL_SW | CON_WAIT)

/* we always read these as a word */
/* Device Type 2 */
#define DEV_AV			(1 << 14)
#define DEV_TTY			(1 << 13)
#define DEV_PPD			(1 << 12)
#define DEV_JIG_UART_OFF	(1 << 11)
#define DEV_JIG_UART_ON		(1 << 10)
#define DEV_JIG_USB_OFF		(1 << 9)
#define DEV_JIG_USB_ON		(1 << 8)
/* Device Type 1 */
#define DEV_USB_OTG		(1 << 7)
#define DEV_DEDICATED_CHG	(1 << 6)
#define DEV_USB_CHG		(1 << 5)
#define DEV_CAR_KIT		(1 << 4)
#define DEV_UART		(1 << 3)
#define DEV_USB			(1 << 2)
#define DEV_AUDIO_2		(1 << 1)
#define DEV_AUDIO_1		(1 << 0)

#define DEV_USB_MASK		(DEV_USB | DEV_JIG_USB_OFF | DEV_JIG_USB_ON)
#define DEV_UART_MASK		(DEV_UART | DEV_JIG_UART_OFF)
#define DEV_JIG_MASK		(DEV_JIG_USB_OFF | DEV_JIG_USB_ON | \
				 DEV_JIG_UART_OFF | DEV_JIG_UART_ON)
#define DEV_CHARGER_MASK	(DEV_DEDICATED_CHG | DEV_USB_CHG | DEV_CAR_KIT)

static int smbus_write_word(uint8_t chip, uint8_t addr, uint16_t value) {
	u8 val[] = {
		(value >> 8) & 0xff,
		value & 0xff
	};

#ifdef DEBUG
	printf("%s: [chip=%x addr=%x value=%x]\n", __func__, chip, addr, value);
#endif //DEBUG

	return i2c_write(chip, addr, 1, val, 2);
}

static int smbus_read_word(uint8_t chip, uint8_t addr, uint16_t *value) {
	u8 val[2];

	int ret = i2c_read(chip, addr, 1, val, 2);
	*value = (val[0] << 8) | val[1];

#ifdef DEBUG
	printf("%s: [chip=%x addr=%x value=%x]\n", __func__, chip, addr, *value);
#endif //DEBUG

	return ret;
}

static inline void fsa9480_print_dev_type(fsa9480_dev_t dev) {
	#define ENUM_ENTRY(x) [x] = #x
	char *map[] = {
		ENUM_ENTRY(FSA9480_UNKNOWN),
		ENUM_ENTRY(FSA9480_USB),
		ENUM_ENTRY(FSA9480_USB_HOST),
		ENUM_ENTRY(FSA9480_CHARGER),
		ENUM_ENTRY(FSA9480_JIG),
		ENUM_ENTRY(FSA9480_UART),
		ENUM_ENTRY(FSA9480_AV_365K),
		ENUM_ENTRY(FSA9480_AV_365K_CHARGER),
	};
	#undef ENUM_ENTRY
	if (sizeof(map) / sizeof(map[0]) <= dev) {
		return;
	}
	printf("FSA9480: device type is %s\n", map[dev]);
}

fsa9480_dev_t fsa9480_detect_device(uint8_t chip_addr) {
	uint16_t dev_type, int_val;
	int i;
	fsa9480_dev_t ret = FSA9480_UNKNOWN;

	for (i = 0; i < 100; i++) {
		int _ok = smbus_read_word(chip_addr, FSA9480_REG_INT1, &int_val);
		if ((_ok < 0) || !int_val)
		{
			break;
		}
	}
	mdelay(1000);

	if (smbus_read_word(chip_addr, FSA9480_REG_DEV_T1, &dev_type) < 0) {
		printf("%s: failed to get device type\n", __func__);
		goto fail;
	}

	if (dev_type & DEV_USB_MASK) {
		ret = FSA9480_USB;
	}
	else if (dev_type & DEV_UART_MASK) {
		ret = FSA9480_UART;
	}
	else if (dev_type & DEV_JIG_MASK) {
		ret = FSA9480_JIG;
	}
	else if (dev_type & DEV_CHARGER_MASK) {
		ret = FSA9480_CHARGER;
	}
	else if (dev_type & DEV_USB_OTG) {
		ret = FSA9480_USB_HOST;
	}

	fsa9480_print_dev_type(ret);

fail:
	return ret;
}

int fsa9480_probe(uint8_t chip_addr) {
	int i;

	struct {
		uint8_t reg;
		uint16_t value;
	} regs[] = {
		{
			.reg = FSA9480_REG_INT1_MASK,
			.value = 0x1fff,
		},
		{
			.reg = FSA9480_REG_CK_INTMASK1,
			.value = 0x7ff
		},
		{
			.reg = FSA9480_REG_TIMING1,
			.value = 500,
		},
		{
			.reg = FSA9480_REG_MANSW1,
			.value = 0,
		},
	};
	

	for (i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
		if (smbus_write_word(chip_addr, regs[i].reg, regs[i].value) < 0) {
			printf("%s: failed to write %x -> %x\n",
				__func__, regs[i].reg, regs[i].value);
			return -1;
		}
	}
	return 0;
}

/******************************************************************************
 * Tuna-specific FSA9480 handling ported from linux
 *****************************************************************************/
#define TUNA_FSA9480_BUS 3
#define TUNA_FSA9480_ADDR (0x4a >> 1)

#define TUNA_GPIO_CP_USB_ON	22
#define TUNA_GPIO_MHL_SEL		96
#define TUNA_GPIO_AP_SEL		97
#define TUNA_GPIO_MUX3_SEL0	139
#define TUNA_GPIO_MUX3_SEL1 140
#define TUNA_GPIO_IF_UART_SEL 101
#define TUNA_GPIO_USB_ID_SEL		191

#define MUX3_SEL0_AP		1
#define MUX3_SEL1_AP		1
#define MUX3_SEL0_MHL		1
#define MUX3_SEL1_MHL		0
#define MUX3_SEL0_FSA		0
#define MUX3_SEL1_FSA		1

#define FSA3200_AP_SEL_AP	0
#define FSA3200_MHL_SEL_AP	0
#define FSA3200_AP_SEL_FSA	1
#define FSA3200_MHL_SEL_FSA	0
#define FSA3200_AP_SEL_MHL	1
#define FSA3200_MHL_SEL_MHL	1

#define USB_ID_SEL_FSA		0
#define USB_ID_SEL_MHL		1

#define IF_UART_SEL_DEFAULT	1
#define IF_UART_SEL_AP		1
#define IF_UART_SEL_CP		0

enum {
	TUNA_USB_MUX_FSA = 0,
	TUNA_USB_MUX_MHL,
	TUNA_USB_MUX_AP,
	NUM_TUNA_USB_MUX,

	TUNA_USB_MUX_DEFAULT = TUNA_USB_MUX_FSA,
};

static struct {
	int mux3_sel0;
	int mux3_sel1;
} tuna_usb_mux_states[] = {
	[TUNA_USB_MUX_FSA] = { MUX3_SEL0_FSA, MUX3_SEL1_FSA },
	[TUNA_USB_MUX_MHL] = { MUX3_SEL0_MHL, MUX3_SEL1_MHL },
	[TUNA_USB_MUX_AP] = { MUX3_SEL0_AP, MUX3_SEL1_AP },
};

static struct {
	int ap_sel;
	int mhl_sel;
} tuna_fsa3200_mux_pair_states[] = {
	[TUNA_USB_MUX_FSA] = { FSA3200_AP_SEL_FSA, FSA3200_MHL_SEL_FSA },
	[TUNA_USB_MUX_MHL] = { FSA3200_AP_SEL_MHL, FSA3200_MHL_SEL_MHL },
	[TUNA_USB_MUX_AP] = { FSA3200_AP_SEL_AP, FSA3200_MHL_SEL_AP },
};

static int tuna_usb_id_mux_states[] = {
	[TUNA_USB_MUX_FSA] = USB_ID_SEL_FSA,
	[TUNA_USB_MUX_MHL] = USB_ID_SEL_MHL,
	[TUNA_USB_MUX_AP] = USB_ID_SEL_FSA,
};

static void tuna_mux_usb(int state)
{
	printf("%s: mux to %d\n", __func__, state);
	gpio_direction_output(TUNA_GPIO_MUX3_SEL0,
			      tuna_usb_mux_states[state].mux3_sel0);
	gpio_direction_output(TUNA_GPIO_MUX3_SEL1,
			      tuna_usb_mux_states[state].mux3_sel1);
	gpio_set_value(TUNA_GPIO_MUX3_SEL0,
			      tuna_usb_mux_states[state].mux3_sel0);
	gpio_set_value(TUNA_GPIO_MUX3_SEL1,
			      tuna_usb_mux_states[state].mux3_sel1);
}

static void tuna_mux_usb_id(int state)
{
	printf("%s: mux to %d\n", __func__, state);
	gpio_direction_output(TUNA_GPIO_USB_ID_SEL,
		tuna_usb_id_mux_states[state]);
	gpio_set_value(TUNA_GPIO_USB_ID_SEL,
		tuna_usb_id_mux_states[state]);
}

static void tuna_fsa3200_mux_pair(int state)
{
	printf("%s: mux to %d\n", __func__, state);
	gpio_direction_output(TUNA_GPIO_AP_SEL,
			      tuna_fsa3200_mux_pair_states[state].ap_sel);
	gpio_direction_output(TUNA_GPIO_MHL_SEL,
			      tuna_fsa3200_mux_pair_states[state].mhl_sel);
	gpio_set_value(TUNA_GPIO_AP_SEL,
			      tuna_fsa3200_mux_pair_states[state].ap_sel);
	gpio_set_value(TUNA_GPIO_MHL_SEL,
			      tuna_fsa3200_mux_pair_states[state].mhl_sel);
}

static void tuna_mux_usb_to_fsa(int enable)
{
	if (omap4_tuna_get_revision() >= 3) {
		tuna_fsa3200_mux_pair(enable ? TUNA_USB_MUX_FSA :
				TUNA_USB_MUX_DEFAULT);
	} else {
		tuna_mux_usb(enable ? TUNA_USB_MUX_FSA : TUNA_USB_MUX_DEFAULT);

		/* When switching ID away from FSA, we want to ensure we switch
		 * it off FSA, and force it to MHL. Ideally, we'd just say mux
		 * to default, but FSA is likely the default mux position and
		 * there's no way to force the ID pin to float to the FSA.
		 */
		tuna_mux_usb_id(enable ? TUNA_USB_MUX_FSA : TUNA_USB_MUX_MHL);
	}
}

static void tuna_ap_usb_attach(void)
{
	if (omap4_tuna_get_revision() >= 3) {
		tuna_fsa3200_mux_pair(TUNA_USB_MUX_AP);
	} else {
		tuna_mux_usb(TUNA_USB_MUX_AP);
		tuna_mux_usb_id(TUNA_USB_MUX_FSA);
	}
}

static void fsa9480_init(void) {
	tuna_clear_i2c4();
	i2c_set_bus_num(TUNA_FSA9480_BUS);

	fsa9480_probe(TUNA_FSA9480_ADDR);
	fsa9480_dev_t fsa_dev_type = fsa9480_detect_device(TUNA_FSA9480_ADDR);

	i2c_set_bus_num(0);

	switch (fsa_dev_type) {
		case FSA9480_USB:
			tuna_mux_usb_to_fsa(0);
			tuna_ap_usb_attach();
			gpio_direction_output(TUNA_GPIO_IF_UART_SEL, 0);
			gpio_set_value(TUNA_GPIO_IF_UART_SEL, 0);
			break;
		case FSA9480_UART:
			tuna_mux_usb_to_fsa(1);
			gpio_direction_output(TUNA_GPIO_IF_UART_SEL, 1);
			gpio_set_value(TUNA_GPIO_IF_UART_SEL, 1);
			break;
		default:
			break;
	}
}


int do_tuna_check_cable(cmd_tbl_t *cmdtp, int flag,
	int argc, char * const argv[])
{
	fsa9480_init();
	return 0;
}

U_BOOT_CMD(tuna_check_cable, CONFIG_SYS_MAXARGS, 1, do_tuna_check_cable,
	"Get Tuna (Galaxy Nexus) cable type\n",
	"tuna_check_cable\n"
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
	set_muxconf_regs_non_essential();
	gpmc_init();

	gd->bd->bi_arch_number = MACH_TYPE_TUNA;
	gd->bd->bi_boot_params = (0x80000000 + 0x100); /* boot param addr */
	gd->ram_size = 0x40000000;
	
	gd->bd->bi_dram[0].start = 0x80000000;
	gd->bd->bi_dram[0].size =  0x40000000;

	//get it before board-specific hardware initialization routines are called
	gnex_get_revision();
	
	//indicate we're alive
	tuna_set_led(5);
	tuna_check_bootflag();

	//udc_init();

	return 0;
}

#if defined(CONFIG_USB_ETHER)
int board_eth_init(bd_t *bis)
{
	cpu_eth_init(bis);
	#if defined(CONFIG_MUSB_GADGET)
		return usb_eth_initialize(bis);
	#else
		return 0;
	#endif
}
#endif

#ifdef CONFIG_USB_MUSB_OMAP2PLUS
static struct musb_hdrc_config musb_config = {
	.multipoint     = 1,
	.dyn_fifo       = 1,
	.num_eps        = 16,
	.ram_bits       = 12,
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
};

static struct musb_hdrc_platform_data musb_plat = {
	.mode		= MUSB_PERIPHERAL,
	.config         = &musb_config,
	.power          = 100,
	.platform_ops	= &omap2430_ops,
	.board_data	= &musb_board_data,
};
#endif

/**
 * @brief misc_init_r - set up tuna misc hardware (currently ULPI PHY clock)
 * @return 0
 */
int misc_init_r(void)
{
#if 0
	u32 auxclk, altclksrc;

	gpio_direction_output(TUNA_GPIO_USB3333_RESETB, 0);
	gpio_set_value(TUNA_GPIO_USB3333_RESETB, 0);

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

	altclksrc = readl(&scrm->altclksrc);

	/* Activate alternate system clock supplier */
	altclksrc &= ~ALTCLKSRC_MODE_MASK;
	altclksrc |= ALTCLKSRC_MODE_ACTIVE;

	/* enable clocks */
	altclksrc |= ALTCLKSRC_ENABLE_INT_MASK | ALTCLKSRC_ENABLE_EXT_MASK;
	writel(altclksrc, &scrm->altclksrc);
	
	mdelay(1);
	gpio_set_value(TUNA_GPIO_USB3333_RESETB, 1);
#endif	

#ifdef CONFIG_USB_MUSB_OMAP2PLUS
	#define SCM_BASE 0x4a002000

	u32 tmp;
	
	tmp = readl(SCM_BASE + 0x300);
	writel(tmp & ~1, SCM_BASE + 0x300);

	tmp = readl(SCM_BASE + 0x33c);
	tmp |= (1 | (1 << 2));
	tmp &= ~((1 << 3) | (1 << 4));
	writel(tmp, SCM_BASE + 0x33c);

	musb_register(&musb_plat, &musb_board_data, (void *)MUSB_BASE);
#endif
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

void board_usb_init(void) {
	printf("%s\n", __func__);
	//musb_register(&musb_plat, &musb_board_data, (void *)MUSB_BASE);
}

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
