/*
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio_event.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/led-lm3559.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <media/omap4iss.h>
#include <media/ov5640.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/consumer.h>
#include <plat/omap-pm.h>
#include <asm/mach-types.h>

#ifdef CONFIG_REGULATOR_LP8720
#include <linux/regulator/machine.h>
#include <linux/regulator/lp8720.h>
#endif

#include "devices.h"
#include "../../../drivers/media/video/omap4iss/iss.h"
#include "control.h"
#include "mux.h"

#define GPIO_TOUCH_EN           35
#define GPIO_TOUCH_IRQ          119
#define GPIO_LED_EN		191

/* touch is on i2c2 */
#define GPIO_TOUCH_SCL  128
#define GPIO_TOUCH_SDA  129

static struct lm3559_platform_data lm3559_pdata = {
	.gpio_hwen      =       GPIO_LED_EN,
};

static struct i2c_board_info __initdata camera_i2c2_boardinfo_final[] = {
	{
		I2C_BOARD_INFO("lm3559", 0x53),
		.platform_data = &lm3559_pdata,
	},
};

#ifdef CONFIG_REGULATOR_LP8720
static struct regulator_consumer_supply folder_pmic_ldo1_supply[] = {
        REGULATOR_SUPPLY("cam_ldo1", NULL),
};

static struct regulator_consumer_supply folder_pmic_ldo2_supply[] = {
        REGULATOR_SUPPLY("cam_ldo2", NULL),
};

static struct regulator_consumer_supply folder_pmic_ldo3_supply[] = {
        REGULATOR_SUPPLY("cam_ldo3", NULL),
};

static struct regulator_consumer_supply folder_pmic_ldo4_supply[] = {
        REGULATOR_SUPPLY("cam_ldo4", NULL),
};

static struct regulator_consumer_supply folder_pmic_ldo5_supply[] = {
        REGULATOR_SUPPLY("cam_ldo5", NULL),
};

#define REGULATOR_INIT(_ldo, _name, _min_uV, _max_uV, _always_on, _ops_mask, _disabled)	\
	static struct regulator_init_data _ldo##_init_data = {          \
		.constraints = {                                        \
			.name   = _name,                                \
			.min_uV = _min_uV,                              \
			.max_uV = _max_uV,                              \
			.always_on      = _always_on,                   \
			.boot_on        = _always_on,                   \
			.apply_uV       = 1,                            \
			.valid_ops_mask = _ops_mask,                    \
			.state_mem = {                                  \
				.disabled       = _disabled,            \
				.enabled        = !(_disabled),         \
			}                                               \
		},                                                      \
		.num_consumer_supplies = ARRAY_SIZE(_ldo##_supply),     \
		.consumer_supplies = &_ldo##_supply[0],                 \
	};

REGULATOR_INIT(folder_pmic_ldo1, "CAM_LDO1", 1800000, 1800000, 1, REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(folder_pmic_ldo2, "CAM_LDO2", 2800000, 2800000, 1, REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(folder_pmic_ldo3, "CAM_LDO3", 2800000, 2800000, 1, REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(folder_pmic_ldo4, "CAM_LDO4", 1800000, 1800000, 1, REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(folder_pmic_ldo5, "CAM_LDO5", 2800000, 2800000, 1, REGULATOR_CHANGE_STATUS, 1);

static struct lp8720_regulator_subdev lp8720_folder_regulators[] = {
        { LP8720_LDO1, &folder_pmic_ldo1_init_data, },
        { LP8720_LDO2, &folder_pmic_ldo2_init_data, },
        { LP8720_LDO3, &folder_pmic_ldo3_init_data, },
        { LP8720_LDO4, &folder_pmic_ldo4_init_data, },
        { LP8720_LDO5, &folder_pmic_ldo5_init_data, },
};

#define LP8720_GPIO	83
struct lp8720_platform_data folder_pmic_info = {
        .name = "lp8720_folder_pmic",
        .en_pin = LP8720_GPIO,
        .num_regulators = ARRAY_SIZE(lp8720_folder_regulators),
        .regulators = lp8720_folder_regulators,
};

static struct i2c_board_info __initdata camera_pmic_i2c2_sensor_boardinfo[] = {
        {
                I2C_BOARD_INFO("lp8720", 0xFA>>1),
                .platform_data = &folder_pmic_info,
        },
};
#endif

static void __init camera_backlight_init(void)
{
	/**
	 * backlight led
	 */
	gpio_request(GPIO_LED_EN, "lm3559_en");
	gpio_direction_output(GPIO_LED_EN, 1);
	omap_mux_init_gpio(GPIO_LED_EN, OMAP_PIN_OUTPUT);

	i2c_register_board_info(2, camera_i2c2_boardinfo_final, ARRAY_SIZE(camera_i2c2_boardinfo_final));
}

static void __init camera_pmic_init(void)
{
#ifdef CONFIG_REGULATOR_LP8720
        gpio_request(LP8720_GPIO, "lp8720_folder_pmic");
        gpio_direction_output(LP8720_GPIO, 1);
        omap_mux_init_gpio(LP8720_GPIO, OMAP_PIN_OUTPUT);

	i2c_register_board_info(3, camera_pmic_i2c2_sensor_boardinfo, ARRAY_SIZE(camera_pmic_i2c2_sensor_boardinfo));
#endif
}

void __init omap4_camera_input_init(void)
{
	camera_backlight_init();
	camera_pmic_init();
}

#define PANDA_GPIO_CAM_PWRDN            39
#define PANDA_GPIO_CAM_RESET            81

static struct clk *board_44xx_cam_aux_clk1;
static struct clk *board_44xx_cam_aux_clk2;
static struct clk *board_44xx_cam_aux_clk3;
static struct regulator *ov5640_cam2pwr_reg;

static int board_44xx_ov_power(struct v4l2_subdev *subdev, int on)
{
	int ret;
	printk("%s() power: %s\n", __func__, ((on == 1)?"ON":"OFF"));

	if (on) {
		gpio_set_value(PANDA_GPIO_CAM_RESET, 0);
		msleep(5);
		gpio_set_value(PANDA_GPIO_CAM_RESET, 1);

		if (!regulator_is_enabled(ov5640_cam2pwr_reg)) {
			printk("regulator enable\n");
			ret = regulator_enable(ov5640_cam2pwr_reg);
			if (ret) {
				printk("Error in enabling sensor power regulator 'cam2pwr'\n");
				return ret;
			}
			msleep(50);
		}

		gpio_set_value(PANDA_GPIO_CAM_PWRDN, 0);
		ret = clk_enable(board_44xx_cam_aux_clk1);
		if (ret) {
			printk("Error in clk_enable(1) in %s(%d)\n", __func__, on);
			gpio_set_value(PANDA_GPIO_CAM_PWRDN, 1);
			regulator_disable(ov5640_cam2pwr_reg);
			return ret;
		}

		ret = clk_enable(board_44xx_cam_aux_clk2);
		if (ret) {
			printk("Error in clk_enable(2) in %s(%d)\n", __func__, on);
			gpio_set_value(PANDA_GPIO_CAM_PWRDN, 1);
			regulator_disable(ov5640_cam2pwr_reg);
			clk_disable(board_44xx_cam_aux_clk1);
			return ret;
		}
		ret = clk_enable(board_44xx_cam_aux_clk3);
		if (ret) {
			printk("Error in clk_enable(3) in %s(%d)\n", __func__, on);
			gpio_set_value(PANDA_GPIO_CAM_PWRDN, 1);
			regulator_disable(ov5640_cam2pwr_reg);
			clk_disable(board_44xx_cam_aux_clk2);
			clk_disable(board_44xx_cam_aux_clk1);
			return ret;
		}

		mdelay(2);

	} else {
		clk_disable(board_44xx_cam_aux_clk1);
		clk_disable(board_44xx_cam_aux_clk2);
		clk_disable(board_44xx_cam_aux_clk3);
		gpio_set_value(PANDA_GPIO_CAM_PWRDN, 1);
		if (regulator_is_enabled(ov5640_cam2pwr_reg)) {
			printk("regulator disable\n");
			ret = regulator_disable(ov5640_cam2pwr_reg);
			if (ret) {
				printk("Error in disabling sensor power regulator 'cam2pwr'\n");
				return ret;
			}
		}
		gpio_set_value(PANDA_GPIO_CAM_RESET, 0);
	}
	return 0;
}

#define OV5640_I2C_ADDRESS   (0x3C)

static struct ov5640_platform_data ov5640_platform_data = {
      .s_power = board_44xx_ov_power,
};

static struct i2c_board_info ov5640_camera_i2c_device = {
        I2C_BOARD_INFO("ov5640", OV5640_I2C_ADDRESS),
        .platform_data = &ov5640_platform_data,
};

static struct iss_subdev_i2c_board_info ov5640_camera_subdevs[] = {
        {
                .board_info = &ov5640_camera_i2c_device,
                .i2c_adapter_id = 3,
        },
        { NULL, 0, },
};

static struct iss_v4l2_subdevs_group board_44xx_camera_subdevs[] = {
        {
                .subdevs = ov5640_camera_subdevs,
                .interface = ISS_INTERFACE_CSI2A_PHY1,
                .bus = { .csi2 = {
                        .lanecfg        = {
                                .clk = {
                                        .pol = 1,
                                        .pos = 1,
                                },
                                .data[0] = {
                                        .pol = 1,
                                        .pos = 2,
                                },
                                .data[1] = {
                                        .pol = 1,
                                        .pos = 3,
                                },

                        },
                } },
        },
        { },
};

static void board_44xx_omap4iss_set_constraints(struct iss_device *iss, bool enable)
{
        if (!iss)
                return;

        /* FIXME: Look for something more precise as a good throughtput limit */
        omap_pm_set_min_bus_tput(iss->dev, OCP_INITIATOR_AGENT,
                                 enable ? 800000 : -1);
}

static struct iss_platform_data board_44xx_iss_platform_data = {
        .subdevs = board_44xx_camera_subdevs,
        .set_constraints = board_44xx_omap4iss_set_constraints,
};

static struct omap_device_pad omap4iss_pads[] = {
        {
                .name   = "csi21_dx0.csi21_dx0",
                .enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
        },
        {
                .name   = "csi21_dy0.csi21_dy0",
                .enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
        },
        {
                .name   = "csi21_dx1.csi21_dx1",
                .enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
        },
        {
                .name   = "csi21_dy1.csi21_dy1",
                .enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
        },
        {
                .name   = "csi21_dx2.csi21_dx2",
                .enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
        },
        {
                .name   = "csi21_dy2.csi21_dy2",
                .enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
        },
};

static struct omap_board_data omap4iss_data = {
        .id                     = 1,
        .pads                   = omap4iss_pads,
        .pads_cnt               = ARRAY_SIZE(omap4iss_pads),
};

static int __init board_44xx_camera_init(void)
{
	ov5640_cam2pwr_reg = regulator_get(NULL, "cam2pwr");
	if (IS_ERR(ov5640_cam2pwr_reg)) {
		printk(KERN_ERR "Unable to get 'cam2pwr' regulator for sensor power\n");
		return -ENODEV;
	}

	if (regulator_set_voltage(ov5640_cam2pwr_reg, 1800000, 1800000)) {
		printk(KERN_ERR "Unable to set valid 'cam2pwr' regulator"
				" voltage range to: 1.7V ~ 1.8V\n");
		regulator_put(ov5640_cam2pwr_reg);
		return -ENODEV;
	}

	board_44xx_cam_aux_clk1 = clk_get(NULL, "auxclk1_ck");
	if (IS_ERR(board_44xx_cam_aux_clk1)) {
		printk(KERN_ERR "Unable to get auxclk1_ck\n");
		return -ENODEV;
	}
	if (clk_set_rate(board_44xx_cam_aux_clk1, clk_round_rate(board_44xx_cam_aux_clk1, 24000000)))
		return -EINVAL;

	board_44xx_cam_aux_clk2 = clk_get(NULL, "auxclk2_ck");
	if (IS_ERR(board_44xx_cam_aux_clk2)) {
		printk(KERN_ERR "Unable to get auxclk2_ck\n");
		return -ENODEV;
	}
	if (clk_set_rate(board_44xx_cam_aux_clk2, clk_round_rate(board_44xx_cam_aux_clk2, 24000000)))
		return -EINVAL;

	board_44xx_cam_aux_clk3 = clk_get(NULL, "auxclk3_ck");
	if (IS_ERR(board_44xx_cam_aux_clk3)) {
		printk(KERN_ERR "Unable to get auxclk3_ck\n");
		return -ENODEV;
	}
	if (clk_set_rate(board_44xx_cam_aux_clk3, clk_round_rate(board_44xx_cam_aux_clk3, 24000000)))
		return -EINVAL;
	/*
	 * CSI2 1(A):
	 *   LANEENABLE[4:0] = 00111(0x7) - Lanes 0, 1 & 2 enabled
	 *   CTRLCLKEN = 1 - Active high enable for CTRLCLK
	 *   CAMMODE = 0 - DPHY mode
	 */
	omap4_ctrl_pad_writel((omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_CAMERA_RX) &
				~(OMAP4_CAMERARX_CSI21_LANEENABLE_MASK |
					OMAP4_CAMERARX_CSI21_CAMMODE_MASK)) |
			(0x7 << OMAP4_CAMERARX_CSI21_LANEENABLE_SHIFT) |
			OMAP4_CAMERARX_CSI21_CTRLCLKEN_MASK,
			OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_CAMERA_RX);

	/* Select GPIO 45 */
	omap_mux_init_gpio(PANDA_GPIO_CAM_PWRDN, OMAP_PIN_OUTPUT);

	/* Select GPIO 83 */
	omap_mux_init_gpio(PANDA_GPIO_CAM_RESET, OMAP_PIN_OUTPUT);

	/* Init FREF_CLK1_OUT */
	omap_mux_init_signal("fref_clk1_out", OMAP_PIN_OUTPUT);
	if (gpio_request_one(PANDA_GPIO_CAM_PWRDN, GPIOF_OUT_INIT_HIGH,
				"CAM_PWRDN"))
		printk(KERN_WARNING "Cannot request GPIO %d\n",
				PANDA_GPIO_CAM_PWRDN);

	if (gpio_request_one(PANDA_GPIO_CAM_RESET, GPIOF_OUT_INIT_HIGH,
				"CAM_RESET"))
		printk(KERN_WARNING "Cannot request GPIO %d\n",
				PANDA_GPIO_CAM_RESET);

	gpio_set_value(PANDA_GPIO_CAM_RESET, 0);
	msleep(5);
	gpio_set_value(PANDA_GPIO_CAM_RESET, 1);

	omap4_init_camera(&board_44xx_iss_platform_data, &omap4iss_data);
	return 0;
}
        
late_initcall(board_44xx_camera_init);
