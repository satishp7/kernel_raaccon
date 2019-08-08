/*
 * Driver for S5K5BAFX (VGA camera) from SAMSUNG ELECTRONICS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <media/v4l2-subdev.h>

struct s5k5baf_platform_data {
	u32 default_width;
	u32 default_height;
	u32 pixelformat;
	u32 freq;	/* MCLK in KHz */

	/* This SoC supports Parallel & CSI-2 */
	u32 is_mipi;		/* set to 1 if mipi */
	bool init_streamoff;

    int (*s_power)(struct v4l2_subdev *subdev, int on);
};
