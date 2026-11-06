/*
 * Copyright (C) 2022-2023, Xiaomi, Inc.
 * All Rights Reserved.
 */

#include "fp_driver.h"
#include <linux/pinctrl/consumer.h>

/* -------------------------------------------------------------------- */
/* fingerprint chip hardware configuration				*/
/* -------------------------------------------------------------------- */
#define DTS_VOlT_REGULATER			"vfp"
#define DTS_NETLINK_NUM				"netlink-event"
#define DTS_IRQ_GPIO				"xiaomi,gpio_irq"
#define DTS_PINCTL_RESET_HIGH			"reset_high"
#define DTS_PINCTL_RESET_LOW			"reset_low"

int fp_parse_dts(struct fp_device *fp_dev)
{
	struct device *dev = &fp_dev->driver_device->dev;
	struct device_node *node = dev->of_node;
	int ret;

	if (!node)
		return -ENODEV;
	fp_dev->irq_gpio = of_get_named_gpio(node, DTS_IRQ_GPIO, 0);
	if (!gpio_is_valid(fp_dev->irq_gpio))
		return fp_dev->irq_gpio < 0 ? fp_dev->irq_gpio : -EINVAL;
	fp_dev->irq_num = gpio_to_irq(fp_dev->irq_gpio);
	if (fp_dev->irq_num < 0)
		return fp_dev->irq_num;
	ret = of_property_read_u32(node, DTS_NETLINK_NUM, &fp_dev->netlink_num);
	if (ret)
		return ret;
	if (!fp_dev->netlink_num || fp_dev->netlink_num >= MAX_LINKS)
		return -EINVAL;

	fp_dev->vreg = devm_regulator_get(dev, DTS_VOlT_REGULATER);
	if (IS_ERR(fp_dev->vreg))
		return PTR_ERR(fp_dev->vreg);
	fp_dev->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(fp_dev->pinctrl))
		return PTR_ERR(fp_dev->pinctrl);
#define FP_LOOKUP_STATE(member, name) do { \
	fp_dev->member = pinctrl_lookup_state(fp_dev->pinctrl, name); \
	if (IS_ERR(fp_dev->member)) \
		return PTR_ERR(fp_dev->member); \
} while (0)
	FP_LOOKUP_STATE(pins_spiio_spi_mode, "spiio_spi_mode");
	FP_LOOKUP_STATE(pins_spiio_gpio_mode, "spiio_gpio_mode");
	FP_LOOKUP_STATE(pins_reset_high, DTS_PINCTL_RESET_HIGH);
	FP_LOOKUP_STATE(pins_reset_low, DTS_PINCTL_RESET_LOW);
	FP_LOOKUP_STATE(pins_eint_default, "eint_default");
	FP_LOOKUP_STATE(pins_eint_pulldown, "eint_pulldown");
#undef FP_LOOKUP_STATE
	/* Stock Rodin holds SPI in GPIO mode until the supply is enabled. */
	ret = pinctrl_select_state(fp_dev->pinctrl, fp_dev->pins_spiio_gpio_mode);
	if (ret)
		return ret;
	ret = pinctrl_select_state(fp_dev->pinctrl, fp_dev->pins_reset_low);
	if (ret)
		return ret;
	return pinctrl_select_state(fp_dev->pinctrl, fp_dev->pins_eint_pulldown);
}

int fp_power_on(struct fp_device *fp_dev)
{
	int ret;

	if (fp_dev->regulator_enabled) {
		fp_dev->device_available = 1;
		return 0;
	}
	ret = regulator_set_voltage(fp_dev->vreg, 3300000, 3300000);
	if (ret)
		return ret;
	ret = regulator_enable(fp_dev->vreg);
	if (ret)
		return ret;
	fp_dev->regulator_enabled = true;
	fp_dev->device_available = 1;
	return 0;
}

int fp_power_off(struct fp_device *fp_dev)
{
	int ret;

	if (!fp_dev->regulator_enabled) {
		fp_dev->device_available = 0;
		return 0;
	}
	ret = regulator_disable(fp_dev->vreg);
	if (ret)
		return ret;
	fp_dev->regulator_enabled = false;
	fp_dev->device_available = 0;
	return 0;
}

/* delay ms after reset */
void fp_hw_reset(struct fp_device *fp_dev, u8 delay_ms)
{
	pinctrl_select_state(fp_dev->pinctrl, fp_dev->pins_reset_low);
	mdelay(5);
	pinctrl_select_state(fp_dev->pinctrl, fp_dev->pins_reset_high);
	mdelay(delay_ms);
}

void fp_enable_irq(struct fp_device *fp_dev)
{
	if (!fp_dev->irq_requested)
		return;
	if (1 == fp_dev->irq_enabled) {
		pr_debug( "irq already enabled\n");
	} else {
		enable_irq(fp_dev->irq_num);
		fp_dev->irq_enabled = 1;
		pr_debug( "enable irq!\n");
	}
}

void fp_disable_irq(struct fp_device *fp_dev)
{
	if (!fp_dev->irq_requested)
		return;
	if (0 == fp_dev->irq_enabled) {
		pr_debug( "irq already disabled\n");
	} else {
		disable_irq(fp_dev->irq_num);
		fp_dev->irq_enabled = 0;
		pr_debug("disable irq!\n");
	}
}

void fp_kernel_key_input(struct fp_device *fp_dev, struct fp_key *fp_key)
{
	uint32_t key_input = 0;

	if (FP_KEY_HOME == fp_key->key) {
		key_input = FP_KEY_INPUT_HOME;
	} else if (FP_KEY_POWER == fp_key->key) {
		key_input = FP_KEY_INPUT_POWER;
	} else if (FP_KEY_CAMERA == fp_key->key) {
		key_input = FP_KEY_INPUT_CAMERA;
	} else {
		/* add special key define */
		key_input = fp_key->key;
	}

	pr_debug("received key event[%d], key=%d, value=%d\n",
		 key_input, fp_key->key, fp_key->value);

	if ((FP_KEY_POWER == fp_key->key || FP_KEY_CAMERA == fp_key->key)
		&& (fp_key->value == 1)) {
		input_report_key(fp_dev->input, key_input, 1);
		input_sync(fp_dev->input);
		input_report_key(fp_dev->input, key_input, 0);
		input_sync(fp_dev->input);
	}

	if (FP_KEY_HOME == fp_key->key) {
		input_report_key(fp_dev->input, key_input, fp_key->value);
		input_sync(fp_dev->input);
	}
}
