#ifndef __HX8369_H__
#define __HX8369_H__ __FILE__

/* Backlight Level ************************************************************/
#define MIN_BL	30
#define DIM_BL	20
#define MAX_BL	255

#define LOW_BRIGHTNESS_LEVEL 30
#define MAX_BRIGHTNESS_LEVEL 255

#define LOW_BACKLIGHT_VALUE_SONY 13
#define DIM_BACKLIGHT_VALUE_SONY 13
#define MAX_BACKLIGHT_VALUE_SONY 183

/* CABC ***********************************************************************/
#ifdef CONFIG_FB_S3C_HX8369_CAB
typedef enum
{
	CABC_OFF,
	CABC_IMAGE,
	CABC_VIDEO,
} hx8369_cab_t;
#endif

/******************************************************************************/
#define SLEEPMSEC		0x1000
#define ENDDEF			0x2000
#define	DEFMASK			0xFF00

struct s5p_tft_panel_data {
	const u16 *seq_set;
	const u16 *bl_set;
	const u16 *display_off;
#ifdef CONFIG_FB_S3C_HX8369_CAB
	const u16 *cab_off;
	const u16 *cab_image;
	const u16 *cab_video;
#endif
	u16 *brightness_set;
	int pwm_reg_offset;
};

#ifdef CONFIG_MACH_VENTURI
extern struct s5p_tft_panel_data venturi_panel_data;
#endif

#endif // __HX8369_H__
