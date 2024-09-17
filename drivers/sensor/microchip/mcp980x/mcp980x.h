/*
 * Copyright (c) 2019 Peter Bigot Consulting, LLC
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MCP980X_MCP980X_H_
#define ZEPHYR_DRIVERS_SENSOR_MCP980X_MCP980X_H_

#include <errno.h>

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#if CONFIG_MCP9808
#include "mcp9808.h"
#elif CONFIG_MCP9800
#include "mcp9800.h"
#else
#error "Unsupported MCP chipset"
#endif

struct mcp980x_data {
	uint16_t reg_val;

#ifdef CONFIG_MCP980X_TRIGGER
	struct gpio_callback alert_cb;

	const struct device *dev;

	const struct sensor_trigger *trig;
	sensor_trigger_handler_t trigger_handler;
#endif

#ifdef CONFIG_MCP980X_TRIGGER_OWN_THREAD
	struct k_sem sem;
#endif

#ifdef CONFIG_MCP980X_TRIGGER_GLOBAL_THREAD
	struct k_work work;
#endif
};

struct mcp980x_config {
	struct i2c_dt_spec i2c;
	uint8_t resolution;
#ifdef CONFIG_MCP980X_TRIGGER
	struct gpio_dt_spec int_gpio;
#endif /* CONFIG_MCP980X_TRIGGER */
};

int mcp980x_reg_read(const struct device *dev, uint8_t reg, uint16_t *val);
int mcp980x_reg_write_16bit(const struct device *dev, uint8_t reg,
			    uint16_t val);
int mcp980x_reg_write_8bit(const struct device *dev, uint8_t reg,
			   uint8_t val);

#ifdef CONFIG_MCP980X_TRIGGER
int mcp980x_attr_set(const struct device *dev, enum sensor_channel chan,
		     enum sensor_attribute attr,
		     const struct sensor_value *val);
int mcp980x_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);
int mcp980x_setup_interrupt(const struct device *dev);
#endif /* CONFIG_MCP980X_TRIGGER */

/* Encode a signed temperature in scaled Celsius to the format used in
 * register values.
 */
static inline uint16_t mcp980x_temp_reg_from_signed(int temp)
{
	/* Get the 12-bit 2s complement value */
	uint16_t rv = temp & MCP980X_TEMP_ABS_MASK;

	if (temp < 0) {
		rv |= MCP980X_TEMP_SIGN_BIT;
	}
	return rv;
}

/* Decode a register temperature value to a signed temperature in
 * scaled Celsius.
 */
static inline int mcp980x_temp_signed_from_reg(uint16_t reg)
{
	int rv = reg & MCP980X_TEMP_ABS_MASK;

	if (reg & MCP980X_TEMP_SIGN_BIT) {
		/* Convert 12-bit 2s complement to signed negative
		 * value.
		 */
		rv = -(1U + (rv ^ MCP980X_TEMP_ABS_MASK));
	}
	return rv;
}

#endif /* ZEPHYR_DRIVERS_SENSOR_MCP980X_MCP980X_H_ */
