/*
 * hx8369 TFT-LCD Panel Driver for the Samsung Universal board
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/hx8369.h>
#include <linux/lcd.h>
#include <linux/spi/spi.h>
#include <linux/wait.h>

#include <plat/gpio-cfg.h>

#ifdef CONFIG_MACH_VENTURI
#include <mach/gpio-venturi.h>
#endif

#ifdef CONFIG_FB_S3C_MDNIE
#include "s3cfb_mdnie.h"
#endif

struct s5p_lcd {
	struct device		*dev;
	struct backlight_device	*bl_dev;
	struct lcd_device	*lcd_dev;
	struct spi_device	*spi_dev;
	int			brightness;
	bool			ldi_enable;
	struct mutex		spi_lock;
	struct early_suspend	early_suspend;
	struct s5p_tft_panel_data *data;

#ifdef CONFIG_FB_S3C_HX8369_CAB
	struct class		*cab_class;
	struct device		*cab_dev;
	bool			cab_enable;
	hx8369_cab_t		current_cab;
#endif
};

static void hx8369_spi_write(struct s5p_lcd *lcd, u16 reg)
{
	int ret;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len	= 2,
		.tx_buf	= &reg,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(lcd->spi_dev, &msg);
	if (ret < 0)
		pr_err("%s::%d -> spi_sync failed Err=%d\n", __func__, __LINE__, ret);
}

static void hx8369_spi_send_sequence(struct s5p_lcd *lcd, const u16 *buf)
{
	int i = 0;

	mutex_lock(&lcd->spi_lock);

	while ((buf[i] & DEFMASK) != ENDDEF)
	{
		if ((buf[i] & DEFMASK) == SLEEPMSEC)
		{
			i++;
			if ((buf[i] & DEFMASK) == ENDDEF)
				break;
			if ((buf[i] & DEFMASK) == SLEEPMSEC)
				continue;
			msleep(buf[i]);
		}
		else
			hx8369_spi_write(lcd, buf[i]);
		i++;
	}

	mutex_unlock(&lcd->spi_lock);
}

void cab_onoff(struct s5p_lcd *lcd)
{
#ifdef CONFIG_FB_S3C_MDNIE
	if(lcd->cab_enable)
	{
		switch(current_mDNIe_UI)
		{
			case mDNIe_VIDEO_MODE:
			case mDNIe_VIDEO_WARM_MODE:
			case mDNIe_VIDEO_COLD_MODE:
			case mDNIe_CAMERA_MODE:
			case mDNIe_DMB_MODE:
			case mDNIe_DMB_WARM_MODE:
			case mDNIe_DMB_COLD_MODE:
				if(lcd->current_cab != CABC_VIDEO)
				{
					hx8369_spi_send_sequence(lcd, lcd->data->cab_video);
					lcd->current_cab = CABC_VIDEO;
				}
				break;
			case mDNIe_GALLERY:
			case mDNIe_NAVI:
				if(lcd->current_cab != CABC_IMAGE)
		        	{
					hx8369_spi_send_sequence(lcd, lcd->data->cab_image);
				 	lcd->current_cab = CABC_IMAGE;
				}
				break;
			case mDNIe_UI_MODE:
			default:
				if(lcd->current_cab != CABC_OFF)
				{
					hx8369_spi_send_sequence(lcd, lcd->data->cab_off);
					lcd->current_cab = CABC_OFF;
				}
				break;
		}
	}
	else
#endif
	{
		hx8369_spi_send_sequence(lcd, lcd->data->cab_off);
		lcd->current_cab = CABC_OFF;
	}
}

static void update_brightness(struct s5p_lcd *lcd, int level)
{
	unsigned int led_val;

	if (!lcd->ldi_enable)
		return;

	if (level > MAX_BRIGHTNESS_LEVEL)
		level = MAX_BRIGHTNESS_LEVEL;

	if (level >= LOW_BRIGHTNESS_LEVEL)
		led_val = (level - LOW_BRIGHTNESS_LEVEL) * (MAX_BACKLIGHT_VALUE_SONY - LOW_BACKLIGHT_VALUE_SONY) / \
			(MAX_BRIGHTNESS_LEVEL - LOW_BRIGHTNESS_LEVEL) + LOW_BACKLIGHT_VALUE_SONY;
	else if (level > 0)
		led_val = DIM_BACKLIGHT_VALUE_SONY;
	else
		led_val = level;

	if (led_val > MAX_BACKLIGHT_VALUE_SONY)
		led_val = MAX_BACKLIGHT_VALUE_SONY;

	if (level && !led_val)
		led_val = 1;

	lcd->data->brightness_set[lcd->data->pwm_reg_offset] = 0x100 | (led_val & 0xff);
	hx8369_spi_send_sequence(lcd, lcd->data->brightness_set);

	lcd->brightness = level;
}

static int hx8369_backlight_onoff(struct s5p_lcd *lcd, int onoff)
{
	int err;

	err = gpio_request(GPIO_BACKLIGHT_EN, "GPD0");
	if (err < 0)
	{
		printk("Failed to request GPIO_BACKLIGHT_EN!\n");
		return err;
	}

	s3c_gpio_cfgpin(GPIO_BACKLIGHT_EN, S3C_GPIO_OUTPUT);
	gpio_direction_output(GPIO_BACKLIGHT_EN, (int)onoff);
	s3c_gpio_setpull(GPIO_BACKLIGHT_EN, S3C_GPIO_PULL_NONE);
	gpio_free(GPIO_BACKLIGHT_EN);

	return 0;
}

static void hx8369_ldi_enable(struct s5p_lcd *lcd)
{
	hx8369_spi_send_sequence(lcd, lcd->data->seq_set);
	hx8369_spi_send_sequence(lcd, lcd->data->bl_set);
	update_brightness(lcd, lcd->brightness);
	cab_onoff(lcd);
	hx8369_backlight_onoff(lcd, 1);

	lcd->ldi_enable = 1;
}

static void hx8369_ldi_disable(struct s5p_lcd *lcd)
{
	hx8369_spi_send_sequence(lcd, lcd->data->display_off);
	hx8369_backlight_onoff(lcd, 0);

	lcd->ldi_enable = 0;
}

static int s5p_bl_update_status(struct backlight_device* bd)
{
	struct s5p_lcd *lcd = bl_get_data(bd);
	int bl = bd->props.brightness;

	if(bl < 0 || bl > 0xff)
		return -EINVAL;

	if (!lcd->ldi_enable)
		return -ENODEV;

	update_brightness(lcd, bl);

	return 0;
}

static int s5p_bl_get_brightness(struct backlight_device* bd)
{
	struct s5p_lcd *lcd = bl_get_data(bd);

	return lcd->brightness;
}

static struct backlight_ops s5p_bl_ops = {
	.update_status	= s5p_bl_update_status,
	.get_brightness	= s5p_bl_get_brightness,
};

static int s5p_lcd_set_power(struct lcd_device *ld, int power)
{
	struct s5p_lcd *lcd = lcd_get_data(ld);

	if (power)
		hx8369_ldi_enable(lcd);
	else
		hx8369_ldi_disable(lcd);

	return 0;
}

static int s5p_lcd_check_fb(struct lcd_device *ld, struct fb_info *fi)
{
	return 0;
}

static struct lcd_ops s5p_lcd_ops = {
	.set_power	= s5p_lcd_set_power,
	.check_fb	= s5p_lcd_check_fb,
};

static void hx8369_early_suspend(struct early_suspend *h)
{
	struct s5p_lcd *lcd = container_of(h, struct s5p_lcd, early_suspend);

	hx8369_ldi_disable(lcd);
}

static void hx8369_late_resume(struct early_suspend *h)
{
	struct s5p_lcd *lcd = container_of(h, struct s5p_lcd, early_suspend);

	hx8369_ldi_enable(lcd);
}

#ifdef CONFIG_FB_S3C_HX8369_CAB
static ssize_t cab_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct s5p_lcd *lcd = dev_get_drvdata(dev);

	return sprintf(buf,"%u\n", lcd->cab_enable);
}

static ssize_t cab_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct s5p_lcd *lcd = dev_get_drvdata(dev);
	int value;

	sscanf(buf, "%d", &value);

	if (value < 0 || value > 1)
		return -EINVAL;

	lcd->cab_enable = value;
	cab_onoff(lcd);

	return size;
}

static DEVICE_ATTR(enable, 0664, cab_enable_show, cab_enable_store);
#endif

static int __init hx8396_probe(struct spi_device *spi)
{
	struct s5p_lcd *lcd;
	int ret;

	lcd = kzalloc(sizeof(*lcd), GFP_KERNEL);
	if (!lcd) {
		pr_err("failed to allocate for lcd\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	spi->bits_per_word = 9;
	if (spi_setup(spi)) {
		pr_err("failed to setup spi\n");
		ret = -EINVAL;
		goto err_setup;
	}

	mutex_init(&lcd->spi_lock);
	lcd->spi_dev = spi;
	lcd->dev = &spi->dev;
	lcd->brightness = 255;

	if (!spi->dev.platform_data) {
		dev_err(lcd->dev, "failed to get platform data\n");
		ret = -EINVAL;
		goto err_setup;
	}
	lcd->data = (struct s5p_tft_panel_data *)spi->dev.platform_data;

	if (!lcd->data->seq_set || !lcd->data->bl_set ||
		!lcd->data->display_off || !lcd->data->cab_off ||
		!lcd->data->cab_image || !lcd->data->cab_video ||
		!lcd->data->brightness_set || !lcd->data->pwm_reg_offset) {
		dev_err(lcd->dev, "Invalid platform data\n");
		ret = -EINVAL;
		goto err_setup;
	}

	lcd->bl_dev = backlight_device_register("s5p_bl",
			&spi->dev, lcd, &s5p_bl_ops, NULL);
	if (!lcd->bl_dev) {
		dev_err(lcd->dev, "failed to register backlight\n");
		ret = -EINVAL;
		goto err_setup;
	}

	lcd->lcd_dev = lcd_device_register("s5p_lcd",
			&spi->dev, lcd, &s5p_lcd_ops);
	if (!lcd->lcd_dev) {
		dev_err(lcd->dev, "failed to register lcd\n");
		ret = -EINVAL;
		goto err_setup;
	}

#ifdef CONFIG_FB_S3C_HX8369_CAB
	lcd->cab_class = class_create(THIS_MODULE, "cabset");
	if (!lcd->cab_class) {
		dev_err(lcd->dev, "failed to register cabset class\n");
		ret = -EINVAL;
		goto err_setup;
	}

	lcd->cab_dev = device_create(lcd->cab_class, NULL, 0, lcd, "switch_cabset");
	if (!lcd->bl_dev) {
		dev_err(lcd->dev, "failed to register cabset device\n");
		ret = -EINVAL;
		goto err_setup;
	}
	if (device_create_file(lcd->cab_dev, &dev_attr_enable) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_enable.attr.name);
#endif

	lcd->bl_dev->props.max_brightness = 255;
	lcd->bl_dev->props.brightness = lcd->brightness;
	spi_set_drvdata(spi, lcd);

	lcd->ldi_enable = 1;
	lcd->cab_enable = 1;

	hx8369_backlight_onoff(lcd, 1);

#ifdef CONFIG_FB_S3C_MDNIE
	init_mdnie_class();  //set mDNIe UI mode, Outdoormode
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	lcd->early_suspend.suspend = hx8369_early_suspend;
	lcd->early_suspend.resume = hx8369_late_resume;
	lcd->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	register_early_suspend(&lcd->early_suspend);
#endif
	pr_info("%s successfully probed\n", __func__);

	return 0;

err_setup:
	mutex_destroy(&lcd->spi_lock);
	kfree(lcd);

err_alloc:
	return ret;
}

static struct spi_driver hx8369_driver = {
	.driver = {
		.name	= "hx8369",
		.owner	= THIS_MODULE,
	},
	.probe		= hx8396_probe,
	.remove		= __exit_p(hx8369_remove),
};

static int __init hx8369_init(void)
{
	return spi_register_driver(&hx8369_driver);
}

static void __exit hx8369_exit(void)
{
	spi_unregister_driver(&hx8369_driver);
}

module_init(hx8369_init);
module_exit(hx8369_exit);

MODULE_AUTHOR("SAMSUNG");
MODULE_DESCRIPTION("hx8369 LDI driver");
MODULE_LICENSE("GPL");
