/*
 * Copyright (c) 2019 Peter Bigot Consulting, LLC
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MCP9800_MCP9800_H_
#define ZEPHYR_DRIVERS_SENSOR_MCP9800_MCP9800_H_

#include <errno.h>

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#define MCP9800_REG_TEMP_AMB 0x00
#define MCP9800_REG_CONFIG   0x01
#define MCP9800_REG_HYST     0x02
#define MCP9800_REG_LIMIT    0x03

/* 8 bits control configuration and state.
 *
 * * Bit 0 controls shutdown mode
 * * Bit 1 controls interrupt/comparator mode
 * * Bit 2 controls alert polarity
 * * Bit 3-4 fault queue bits
 * * Bit 5-6 ADC resolution bits
 * * Bit 7 one-shot enable bit
 */
#define MCP9800_CFG_SHUTDOWN       BIT(0)
#define MCP9800_CFG_INT_COMP_MODE  BIT(1)
#define MCP9800_CFG_ALERT_POL      BIT(2)
#define MCP9800_CFG_FAULT_QUEUE    BIT(3) | BIT(4)
#define MCP9800_CFG_ADC_RESOLUTION BIT(5) | BIT(6)
#define MCP9800_CFG_ONE_SHOT       BIT(7)
#define MCP9800_REG_RESOLUTION 1
/* 12 bits are used for temperature and state encoding:
 * * Bits 4..15 encode the temperature in a 2s complement signed value
 *   in Celsius with 1/16 Cel resolution
 * * Bit 16 is set to indicate a negative temperature
 */
#define MCP9800_TEMP_SCALE_CEL 256 /* signed */
#define MCP9800_TEMP_SIGN_BIT  BIT(16)
#define MCP9800_TEMP_ABS_MASK  ((uint16_t)(MCP9800_TEMP_SIGN_BIT - 15U))

struct mcp9800_data {
	uint16_t reg_val;

#ifdef CONFIG_MCP9800_TRIGGER
	struct gpio_callback alert_cb;

	const struct device *dev;

	const struct sensor_trigger *trig;
	sensor_trigger_handler_t trigger_handler;
#endif

#ifdef CONFIG_MCP9800_TRIGGER_OWN_THREAD
	struct k_sem sem;
#endif

#ifdef CONFIG_MCP9800_TRIGGER_GLOBAL_THREAD
	struct k_work work;
#endif
};

struct mcp9800_config {
	struct i2c_dt_spec i2c;
	uint8_t resolution;
#ifdef CONFIG_MCP9800_TRIGGER
	struct gpio_dt_spec int_gpio;
#endif /* CONFIG_MCP9800_TRIGGER */
};

int mcp9800_reg_read(const struct device *dev, uint8_t reg, uint16_t *val);
int mcp9800_reg_write_16bit(const struct device *dev, uint8_t reg, uint16_t val);
int mcp9800_reg_write_8bit(const struct device *dev, uint8_t reg, uint8_t val);

#ifdef CONFIG_MCP9800_TRIGGER
int mcp9800_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr,
		     const struct sensor_value *val);
int mcp9800_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);
int mcp9800_setup_interrupt(const struct device *dev);
#endif /* CONFIG_MCP9800_TRIGGER */

/* Encode a signed temperature in scaled Celsius to the format used in
 * register values.
 */
static inline uint16_t mcp9800_temp_reg_from_signed(int temp)
{
	/* Get the 12-bit 2s complement value */
	uint16_t rv = temp & MCP9800_TEMP_ABS_MASK;

	if (temp < 0) {
		rv |= MCP9800_TEMP_SIGN_BIT;
	}
	return rv;
}

/* Decode a register temperature value to a signed temperature in
 * scaled Celsius.
 */
static inline int mcp9800_temp_signed_from_reg(uint16_t reg)
{
	int rv = reg & MCP9800_TEMP_ABS_MASK;

	if (reg & MCP9800_TEMP_SIGN_BIT) {
		/* Convert 12-bit 2s complement to signed negative
		 * value.
		 */
		rv = -(1U + (rv ^ MCP9800_TEMP_ABS_MASK));
	}
	return rv;
}

#endif /* ZEPHYR_DRIVERS_SENSOR_MCP9800_MCP9800_H_ */
