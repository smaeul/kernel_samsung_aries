/* linux/arch/arm/mach-s5pv210/mach-venturi.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/usb/ch9.h>

#include <mach/adc.h>
#include <mach/cpu-freq-v210.h>
#include <mach/gpio.h>
#include <mach/gpio-settings.h>
#include <mach/gpio-venturi.h>
#include <mach/media.h>
#include <mach/regs-clock.h>

#include <plat/cpu.h>
#include <plat/devs.h>
#include <plat/iic.h>
#include <plat/media.h>
#include <plat/mfc.h>
#include <plat/pm.h>
#include <plat/regs-otg.h>
#include <plat/regs-serial.h>
#include <plat/s5pv210.h>
#include <plat/sdhci.h>

/* UARTs */
/* Following are default values for UCON, ULCON and UFCON UART registers */
#define S5PV210_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define S5PV210_ULCON_DEFAULT	 S3C2410_LCON_CS8

#define S5PV210_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg venturi_uartcfgs[] __initdata = {
	{
		.hwport		= 0,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
	},
	{
		.hwport		= 1,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
	},
#ifndef CONFIG_FIQ_DEBUGGER
	{
		.hwport		= 2,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
	},
#endif
	{
		.hwport		= 3,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
	},
};

/* Reserved memory for media devices */
#define S5PV210_VIDEO_SAMSUNG_MEMSIZE_MFC0 (14 * SZ_1M)
#define S5PV210_VIDEO_SAMSUNG_MEMSIZE_MFC1 (21 * SZ_1M)

static struct s5p_media_device venturi_media_devs[] = {
	[0] = {
		.id = S5P_MDEV_MFC,
		.name = "mfc",
		.bank = 0,
		.memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_MFC0,
		.paddr = 0,
	},
	[1] = {
		.id = S5P_MDEV_MFC,
		.name = "mfc",
		.bank = 1,
		.memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_MFC1,
		.paddr = 0,
	},
};

/* GPIO buttons */
static struct gpio_event_direct_entry venturi_keypad_key_map[] = {
		{
			.gpio	= S5PV210_GPH2(6),
			.code	= KEY_POWER,
		},
#ifndef CONFIG_VENTURI_USA
		{
			.gpio	= S5PV210_GPH3(0),
			.code	= KEY_HOME,
		},
#endif
		{
			.gpio	= S5PV210_GPH3(1),
			.code	= KEY_VOLUMEDOWN,
		},
		{
			.gpio	= S5PV210_GPH3(2),
			.code	= KEY_VOLUMEUP,
		}
};

static struct gpio_event_input_info venturi_keypad_key_info = {
	.info.func = gpio_event_input_func,
	.info.no_suspend = true,
	.debounce_time.tv64 = 5 * NSEC_PER_MSEC,
	.type = EV_KEY,
	.keymap = venturi_keypad_key_map,
	.keymap_size = ARRAY_SIZE(venturi_keypad_key_map)
};

static struct gpio_event_info *venturi_input_info[] = {
	&venturi_keypad_key_info.info,
};

static struct gpio_event_platform_data venturi_input_data = {
	.names = {
		"venturi-keypad",
		NULL,
	},
	.info = venturi_input_info,
	.info_count = ARRAY_SIZE(venturi_input_info),
};

static struct platform_device venturi_device_keypad = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev = {
		.platform_data = &venturi_input_data,
	},
};

/* ADC */
static struct s3c_adc_mach_info s3c_adc_platform __initdata = {
	/* s5pc110 supports 12-bit resolution */
	.delay		= 10000,
	.presc		= 65,
	.resolution	= 12,
};

/* I2C0 */
static struct i2c_board_info i2c_devs0[] __initdata = {
};

/* I2C1 */
#ifdef CONFIG_S3C_DEV_I2C1
static struct i2c_board_info i2c_devs1[] __initdata = {
#ifdef CONFIG_VIDEO_TV20
	{
		I2C_BOARD_INFO("s5p_ddc", 0x3A),
	},
#endif
};
#endif

/* I2C2 */
#ifdef CONFIG_S3C_DEV_I2C2
static struct i2c_board_info i2c_devs2[] __initdata = {
#ifdef CONFIG_TOUCHSCREEN_CYTMA340
	{
		I2C_BOARD_INFO("cytma340", 0x20),
	},
#endif
};
#endif

/* I2C3 */
static struct i2c_gpio_platform_data i2c3_platdata = {
	.sda_pin		= GPIO_MSENSE_SDA_28V,
	.scl_pin		= GPIO_MSENSE_SCL_28V,
	.udelay			= 2, /* 250KHz */
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device venturi_device_i2c3 = {
	.name			= "i2c-gpio",
	.id			= 3,
	.dev.platform_data	= &i2c3_platdata,
};

static struct i2c_board_info i2c_devs3[] __initdata = {
#ifdef CONFIG_SENSORS_MMC328X
	{
		I2C_BOARD_INFO("mmc328x", 0x30),
	},
#endif
};

/* I2C4 */
static struct i2c_gpio_platform_data i2c4_platdata = {
	.sda_pin		= GPIO_AP_SDA_18V,
	.scl_pin		= GPIO_AP_SCL_18V,
	.udelay			= 2, /* 250KHz */
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device venturi_device_i2c4 = {
	.name			= "i2c-gpio",
	.id			= 4,
	.dev.platform_data	= &i2c4_platdata,
};

static struct i2c_board_info i2c_devs4[] __initdata = {
#ifdef CONFIG_SND_SMDKC110_MC1N2
	{
		I2C_BOARD_INFO("mc1n2", 0x3A),
	},
#endif
};

/* I2C5 */
static struct i2c_gpio_platform_data i2c5_platdata = {
	.sda_pin		= GPIO_AP_SDA_28V,
	.scl_pin		= GPIO_AP_SCL_28V,
	.udelay			= 2, /* 250KHz */
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device venturi_device_i2c5 = {
	.name			= "i2c-gpio",
	.id			= 5,
	.dev.platform_data	= &i2c5_platdata,
};

static struct i2c_board_info i2c_devs5[] __initdata = {
#ifdef CONFIG_SENSORS_BMA222
	{
		I2C_BOARD_INFO("bma222", 0x08),
	},
#endif
};

/* I2C6 */
static struct i2c_gpio_platform_data i2c6_platdata = {
	.sda_pin		= GPIO_AP_PMIC_SDA,
	.scl_pin		= GPIO_AP_PMIC_SCL,
	.udelay 		= 2, /* 250KHz */
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device venturi_device_i2c6 = {
	.name			= "i2c-gpio",
	.id			= 6,
	.dev.platform_data	= &i2c6_platdata,
};

static struct i2c_board_info i2c_devs6[] __initdata = {
#ifdef CONFIG_REGULATOR_MAX8998
	{
		I2C_BOARD_INFO("max8998", 0x66),
		.platform_data	= &max8998_pdata,
		.irq		= IRQ_EINT7,
	}, {
		I2C_BOARD_INFO("rtc_max8998", 0x06),
	},
#endif
};

/* I2C7 */
static struct i2c_gpio_platform_data i2c7_platdata = {
	.sda_pin		= GPIO_USB_SDA_28V,
	.scl_pin		= GPIO_USB_SCL_28V,
	.udelay 		= 2, /* 250KHz */
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device venturi_device_i2c7 = {
	.name			= "i2c-gpio",
	.id			= 7,
	.dev.platform_data	= &i2c7_platdata,
};

static struct i2c_board_info i2c_devs7[] __initdata = {
#ifdef CONFIG_USB_SWITCH_FSA9480
	{
		I2C_BOARD_INFO("fsa9480", 0x25),
		.platform_data	= &fsa9480_pdata,
		.irq		= IRQ_EINT(23),
	},
#endif
};

/* I2C8 */
static struct i2c_gpio_platform_data i2c8_platdata = {
	.sda_pin		= GPIO_FM_SDA_28V,
	.scl_pin		= GPIO_FM_SCL_28V,
	.udelay 		= 2, /* 250KHz */
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device venturi_device_i2c8 = {
	.name			= "i2c-gpio",
	.id			= 8,
	.dev.platform_data	= &i2c8_platdata,
};

static struct i2c_board_info i2c_devs8[] __initdata = {
#ifdef CONFIG_SAMSUNG_FM_SI4709
	{
		I2C_BOARD_INFO("Si4709", 0x10),
		.irq		= (IRQ_EINT_GROUP20_BASE + 4), /* J2_4 */
	},
#endif
};

/* RAM Console */
static unsigned int ram_console_start;
static unsigned int ram_console_size;

static struct resource ram_console_resource[] = {
	{
		.flags = IORESOURCE_MEM,
	}
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.num_resources = ARRAY_SIZE(ram_console_resource),
	.resource = ram_console_resource,
};

static void __init setup_ram_console_mem(void)
{
	ram_console_resource[0].start = ram_console_start;
	ram_console_resource[0].end = ram_console_start + ram_console_size - 1;
}

/* GPIO */
static void config_gpio_table(int array_size, unsigned int (*gpio_table)[4])
{
	u32 i, gpio;

	for (i = 0; i < array_size; i++) {
		gpio = gpio_table[i][0];
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(gpio_table[i][1]));
		if (gpio_table[i][2] != S3C_GPIO_SETPIN_NONE)
			gpio_set_value(gpio, gpio_table[i][2]);
		s3c_gpio_setpull(gpio, gpio_table[i][3]);
	}
}

static void config_sleep_gpio_table(int array_size, unsigned int (*gpio_table)[3])
{
	u32 i, gpio;

	for (i = 0; i < array_size; i++) {
		gpio = gpio_table[i][0];
		s3c_gpio_slp_cfgpin(gpio, gpio_table[i][1]);
		s3c_gpio_slp_setpull_updown(gpio, gpio_table[i][2]);
	}
}

static void config_init_gpio(void)
{
	config_gpio_table(ARRAY_SIZE(initial_gpio_table), initial_gpio_table);
}

void s3c_config_sleep_gpio(void)
{
	config_gpio_table(ARRAY_SIZE(sleep_alive_gpio_table), sleep_alive_gpio_table);
	config_sleep_gpio_table(ARRAY_SIZE(sleep_gpio_table), sleep_gpio_table);
}
EXPORT_SYMBOL(s3c_config_sleep_gpio);

void s3c_setup_uart_cfg_gpio(unsigned char port)
{
	switch (port) {
	case 0:
		s3c_gpio_cfgpin(GPIO_BT_RXD, S3C_GPIO_SFN(GPIO_BT_RXD_AF));
		s3c_gpio_setpull(GPIO_BT_RXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_BT_TXD, S3C_GPIO_SFN(GPIO_BT_TXD_AF));
		s3c_gpio_setpull(GPIO_BT_TXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_BT_CTS, S3C_GPIO_SFN(GPIO_BT_CTS_AF));
		s3c_gpio_setpull(GPIO_BT_CTS, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_BT_RTS, S3C_GPIO_SFN(GPIO_BT_RTS_AF));
		s3c_gpio_setpull(GPIO_BT_RTS, S3C_GPIO_PULL_NONE);
		s3c_gpio_slp_cfgpin(GPIO_BT_RXD, S3C_GPIO_SLP_PREV);
		s3c_gpio_slp_setpull_updown(GPIO_BT_RXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_slp_cfgpin(GPIO_BT_TXD, S3C_GPIO_SLP_PREV);
		s3c_gpio_slp_setpull_updown(GPIO_BT_TXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_slp_cfgpin(GPIO_BT_CTS, S3C_GPIO_SLP_PREV);
		s3c_gpio_slp_setpull_updown(GPIO_BT_CTS, S3C_GPIO_PULL_NONE);
		s3c_gpio_slp_cfgpin(GPIO_BT_RTS, S3C_GPIO_SLP_PREV);
		s3c_gpio_slp_setpull_updown(GPIO_BT_RTS, S3C_GPIO_PULL_NONE);
		break;
	case 1:
		s3c_gpio_cfgpin(GPIO_GPS_RXD, S3C_GPIO_SFN(GPIO_GPS_RXD_AF));
		s3c_gpio_setpull(GPIO_GPS_RXD, S3C_GPIO_PULL_UP);
		s3c_gpio_cfgpin(GPIO_GPS_TXD, S3C_GPIO_SFN(GPIO_GPS_TXD_AF));
		s3c_gpio_setpull(GPIO_GPS_TXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_GPS_CTS, S3C_GPIO_SFN(GPIO_GPS_CTS_AF));
		s3c_gpio_setpull(GPIO_GPS_CTS, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_GPS_RTS, S3C_GPIO_SFN(GPIO_GPS_RTS_AF));
		s3c_gpio_setpull(GPIO_GPS_RTS, S3C_GPIO_PULL_NONE);
		break;
	case 2:
		s3c_gpio_cfgpin(GPIO_AP_RXD, S3C_GPIO_SFN(GPIO_AP_RXD_AF));
		s3c_gpio_setpull(GPIO_AP_RXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_AP_TXD, S3C_GPIO_SFN(GPIO_AP_TXD_AF));
		s3c_gpio_setpull(GPIO_AP_TXD, S3C_GPIO_PULL_NONE);
		break;
	case 3:
		s3c_gpio_cfgpin(GPIO_FLM_RXD, S3C_GPIO_SFN(GPIO_FLM_RXD_AF));
		s3c_gpio_setpull(GPIO_FLM_RXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_FLM_TXD, S3C_GPIO_SFN(GPIO_FLM_TXD_AF));
		s3c_gpio_setpull(GPIO_FLM_TXD, S3C_GPIO_PULL_NONE);
		break;
	default:
		break;
	}
}
EXPORT_SYMBOL(s3c_setup_uart_cfg_gpio);

/* USB PHY */
#ifdef CONFIG_USB_SUPPORT
/* Initializes OTG Phy. */
void otg_phy_init(void)
{
	/* USB PHY0 Enable */
	writel(readl(S5P_USB_PHY_CONTROL) | (0x1<<0),
			S5P_USB_PHY_CONTROL);
	writel((readl(S3C_USBOTG_PHYPWR) & ~(0x3<<3) & ~(0x1<<0)) | (0x1<<5),
			S3C_USBOTG_PHYPWR);
	writel((readl(S3C_USBOTG_PHYCLK) & ~(0x5<<2)) | (0x3<<0),
			S3C_USBOTG_PHYCLK);
	writel((readl(S3C_USBOTG_RSTCON) & ~(0x3<<1)) | (0x1<<0),
			S3C_USBOTG_RSTCON);
	msleep(1);
	writel(readl(S3C_USBOTG_RSTCON) & ~(0x7<<0),
			S3C_USBOTG_RSTCON);
	msleep(1);

	/* rising/falling time */
	writel(readl(S3C_USBOTG_PHYTUNE) | (0x1<<20),
			S3C_USBOTG_PHYTUNE);

	/* set DC level as 0xf (24%) */
	writel(readl(S3C_USBOTG_PHYTUNE) | 0xf, S3C_USBOTG_PHYTUNE);
}
EXPORT_SYMBOL(otg_phy_init);

/* USB Control request data struct must be located here for DMA transfer */
struct usb_ctrlrequest usb_ctrl __attribute__((aligned(64)));

/* OTG PHY Power Off */
void otg_phy_off(void)
{
	writel(readl(S3C_USBOTG_PHYPWR) | (0x3<<3),
			S3C_USBOTG_PHYPWR);
	writel(readl(S5P_USB_PHY_CONTROL) & ~(1<<0),
			S5P_USB_PHY_CONTROL);
}
EXPORT_SYMBOL(otg_phy_off);

void usb_host_phy_init(void)
{
	struct clk *otg_clk;

	otg_clk = clk_get(NULL, "otg");
	clk_enable(otg_clk);

	if (readl(S5P_USB_PHY_CONTROL) & (0x1<<1))
		return;

	__raw_writel(__raw_readl(S5P_USB_PHY_CONTROL) | (0x1<<1),
			S5P_USB_PHY_CONTROL);
	__raw_writel((__raw_readl(S3C_USBOTG_PHYPWR)
			& ~(0x1<<7) & ~(0x1<<6)) | (0x1<<8) | (0x1<<5),
			S3C_USBOTG_PHYPWR);
	__raw_writel((__raw_readl(S3C_USBOTG_PHYCLK) & ~(0x1<<7)) | (0x3<<0),
			S3C_USBOTG_PHYCLK);
	__raw_writel((__raw_readl(S3C_USBOTG_RSTCON)) | (0x1<<4) | (0x1<<3),
			S3C_USBOTG_RSTCON);
	__raw_writel(__raw_readl(S3C_USBOTG_RSTCON) & ~(0x1<<4) & ~(0x1<<3),
			S3C_USBOTG_RSTCON);
}
EXPORT_SYMBOL(usb_host_phy_init);


void usb_host_phy_off(void)
{
	__raw_writel(__raw_readl(S3C_USBOTG_PHYPWR) | (0x1<<7)|(0x1<<6),
			S3C_USBOTG_PHYPWR);
	__raw_writel(__raw_readl(S5P_USB_PHY_CONTROL) & ~(1<<1),
			S5P_USB_PHY_CONTROL);
}
EXPORT_SYMBOL(usb_host_phy_off);
#endif

/* CPUFreq */
#ifdef CONFIG_CPU_FREQ
static struct s5pv210_cpufreq_voltage smdkc110_cpufreq_volt[] = {
	{
		.freq	= 1200000,
		.varm	= 1275000,
		.vint	= 1100000,
	}, {
		.freq	= 1000000,
		.varm	= 1275000,
		.vint	= 1100000,
	}, {
		.freq	=  800000,
		.varm	= 1200000,
		.vint	= 1100000,
	}, {
		.freq	=  400000,
		.varm	= 1050000,
		.vint	= 1100000,
	}, {
		.freq	=  200000,
		.varm	=  950000,
		.vint	= 1100000,
	}, {
		.freq	=  100000,
		.varm	=  950000,
		.vint	= 1000000,
	},
};

static struct s5pv210_cpufreq_data smdkc110_cpufreq_plat = {
	.volt	= smdkc110_cpufreq_volt,
	.size	= ARRAY_SIZE(smdkc110_cpufreq_volt),
};
#endif

/* PM */
#ifdef CONFIG_S5PV210_PM
unsigned int pm_debug_scratchpad;
#endif

/* Watchdog */
static struct platform_device watchdog_device = {
	.name = "watchdog",
	.id = -1,
};

/* Init data */
static struct platform_device *venturi_devices[] __initdata = {
	&ram_console_device,
	&s3c_device_adc,
#ifdef CONFIG_USB_ANDROID
	&s3c_device_android_usb,
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	&s3c_device_usb_mass_storage,
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	&s3c_device_rndis,
#endif
#endif
	&s3c_device_g3d,
#ifdef CONFIG_S3C_DEV_HSMMC
	&s3c_device_hsmmc0,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	&s3c_device_hsmmc1,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	&s3c_device_hsmmc2,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	&s3c_device_hsmmc3,
#endif
	&s3c_device_i2c0,
#ifdef CONFIG_S3C_DEV_I2C1
	&s3c_device_i2c1,
#endif
#ifdef CONFIG_S3C_DEV_I2C2
	&s3c_device_i2c2,
#endif
	&s3c_device_lcd,
	&s3c_device_mfc,
#ifdef CONFIG_HAVE_PWM
	&s3c_device_timer[0],
	&s3c_device_timer[1],
	&s3c_device_timer[2],
	&s3c_device_timer[3],
#endif
#ifdef CONFIG_USB_GADGET
	&s3c_device_usbgadget,
#endif
	&s3c_device_wdt,
	&s5p_device_onenand,
#ifdef CONFIG_SND_S5P_RP
	&s5p_device_rp,
#endif
#ifdef CONFIG_RTC_DRV_S3C
	&s5p_device_rtc,
#endif
#ifdef CONFIG_CPU_FREQ
	&s5pv210_device_cpufreq,
#endif
#ifdef CONFIG_FIQ_DEBUGGER
	&s5pv210_device_fiqdbg_uart2,
#endif
	&s5pv210_device_iis0,
	&s5pv210_device_pcm1,
	&venturi_device_i2c3,
	&venturi_device_i2c4,
	&venturi_device_i2c5,
	&venturi_device_i2c6,
	&venturi_device_i2c7,
	&venturi_device_i2c8,
	&venturi_device_keypad,
	&watchdog_device,
};

static void __init venturi_fixup(struct machine_desc *desc,
		struct tag *tags, char **cmdline,
		struct meminfo *mi)
{
	mi->bank[0].start = 0x30000000;
	mi->bank[0].size = 256 * SZ_1M;

	mi->bank[1].start = 0x40000000;
	mi->bank[1].size = 255 * SZ_1M;

	mi->nr_banks = 2;

	ram_console_start = mi->bank[1].start + mi->bank[1].size;
	ram_console_size = SZ_1M - SZ_4K;
}

static void __init venturi_machine_init(void)
{
	setup_ram_console_mem();

	platform_add_devices(venturi_devices, ARRAY_SIZE(venturi_devices));

	/* Initialise the GPIOs */
	config_init_gpio();

	/* HSMMC */
#ifdef CONFIG_S3C_DEV_HSMMC
	s5pv210_default_sdhci0();
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	s5pv210_default_sdhci1();
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	s5pv210_default_sdhci2();
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	s5pv210_default_sdhci3();
#endif
#ifdef CONFIG_S5PV210_SETUP_SDHCI
	s3c_sdhci_set_platdata();
#endif

	/* I2C */
	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));
#ifdef CONFIG_S3C_DEV_I2C1
	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));
#endif
#ifdef CONFIG_S3C_DEV_I2C2
	s3c_i2c2_set_platdata(NULL);
	i2c_register_board_info(2, i2c_devs2, ARRAY_SIZE(i2c_devs2));
#endif
	i2c_register_board_info(3, i2c_devs3, ARRAY_SIZE(i2c_devs3));
	i2c_register_board_info(4, i2c_devs4, ARRAY_SIZE(i2c_devs4));
	i2c_register_board_info(5, i2c_devs5, ARRAY_SIZE(i2c_devs5));
	i2c_register_board_info(6, i2c_devs6, ARRAY_SIZE(i2c_devs6));
	i2c_register_board_info(7, i2c_devs7, ARRAY_SIZE(i2c_devs7));
	i2c_register_board_info(8, i2c_devs8, ARRAY_SIZE(i2c_devs8));

	/* ADC */
#if defined(CONFIG_S5P_ADC)
	s3c_adc_set_platdata(&s3c_adc_platform);
#endif

	/* PM */
#if defined(CONFIG_S5PV210_PM)
	s3c_pm_init();
#endif

	/* MFC */
#ifdef CONFIG_VIDEO_MFC50
	s3c_mfc_set_platdata(NULL);
#endif

	/* CPUFreq */
#ifdef CONFIG_CPU_FREQ
	s5pv210_cpufreq_set_platdata(&smdkc110_cpufreq_plat);
#endif
}

static void __init venturi_map_io(void)
{
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);
	s5pv210_gpiolib_init();
	s3c24xx_init_uarts(venturi_uartcfgs, ARRAY_SIZE(venturi_uartcfgs));
#ifndef CONFIG_S5P_HIGH_RES_TIMERS
	s5p_set_timer_source(S5P_PWM3, S5P_PWM4);
#endif

	s5p_reserve_bootmem(venturi_media_devs,
		ARRAY_SIZE(venturi_media_devs), S5P_RANGE_MFC);
}

MACHINE_START(VENTURI, "venturi")
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.fixup		= venturi_fixup,
	.init_irq	= s5pv210_init_irq,
	.map_io		= venturi_map_io,
	.init_machine	= venturi_machine_init,
#ifdef CONFIG_S5P_HIGH_RES_TIMERS
	.timer		= &s5p_systimer,
#else
	.timer		= &s5p_timer,
#endif
MACHINE_END
