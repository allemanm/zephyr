/*
 * Copyright (c) 2019 Peter Bigot Consulting, LLC
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MCP980X_MCP9808_H_
#define ZEPHYR_DRIVERS_SENSOR_MCP980X_MCP9808_H_

#include <errno.h>

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#define MCP9808_REG_CONFIG		0x01
#define MCP9808_REG_UPPER_LIMIT		0x02
#define MCP9808_REG_LOWER_LIMIT		0x03
#define MCP9808_REG_CRITICAL		0x04
#define MCP9808_REG_TEMP_AMB		0x05

/* 16 bits control configuration and state.
 *
 * * Bit 0 controls alert signal output mode
 * * Bit 1 controls interrupt polarity
 * * Bit 2 disables upper and lower threshold checking
 * * Bit 3 enables alert signal output
 * * Bit 4 records alert status
 * * Bit 5 records interrupt status
 * * Bit 6 locks the upper/lower window registers
 * * Bit 7 locks the critical register
 * * Bit 8 enters shutdown mode
 * * Bits 9-10 control threshold hysteresis
 */
#define MCP9808_CFG_ALERT_MODE_INT	BIT(0)
#define MCP9808_CFG_ALERT_ENA		BIT(3)
#define MCP9808_CFG_ALERT_STATE		BIT(4)
#define MCP9808_CFG_INT_CLEAR		BIT(5)

/* 16 bits are used for temperature and state encoding:
 * * Bits 0..11 encode the temperature in a 2s complement signed value
 *   in Celsius with 1/16 Cel resolution
 * * Bit 12 is set to indicate a negative temperature
 * * Bit 13 is set to indicate a temperature below the lower threshold
 * * Bit 14 is set to indicate a temperature above the upper threshold
 * * Bit 15 is set to indicate a temperature above the critical threshold
 */
#define MCP9808_TEMP_SCALE_CEL		16 /* signed */
#define MCP9808_TEMP_SIGN_BIT		BIT(12)
#define MCP9808_TEMP_ABS_MASK		((uint16_t)(MCP9808_TEMP_SIGN_BIT - 1U))
#define MCP9808_TEMP_LWR_BIT		BIT(13)
#define MCP9808_TEMP_UPR_BIT		BIT(14)
#define MCP9808_TEMP_CRT_BIT		BIT(15)

#define MCP9808_REG_RESOLUTION          0x08

#define MCP980X_REG_TEMP_AMB MCP9808_REG_TEMP_AMB
#define MCP980X_TEMP_SCALE_CEL MCP9808_TEMP_SCALE_CEL
#define MCP980X_TEMP_ABS_MASK MCP9808_TEMP_ABS_MASK
#define MCP980X_TEMP_SIGN_BIT MCP9808_TEMP_SIGN_BIT

#endif /* ZEPHYR_DRIVERS_SENSOR_MCP980X_MCP9808_H_ */
