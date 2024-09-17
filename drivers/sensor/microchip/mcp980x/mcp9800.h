/*
 * Copyright (c) 2019 Peter Bigot Consulting, LLC
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MCP980X_MCP9800_H_
#define ZEPHYR_DRIVERS_SENSOR_MCP980X_MCP9800_H_

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

/* 12 bits are used for temperature and state encoding:
 * * Bits 4..15 encode the temperature in a 2s complement signed value
 *   in Celsius with 1/16 Cel resolution
 * * Bit 16 is set to indicate a negative temperature
 */
#define MCP9800_TEMP_SCALE_CEL 256 /* signed */
#define MCP9800_TEMP_SIGN_BIT  BIT(16)
#define MCP9800_TEMP_ABS_MASK  ((uint16_t)(MCP9800_TEMP_SIGN_BIT - 15U))


#define MCP980X_REG_TEMP_AMB MCP9800_REG_TEMP_AMB
#define MCP980X_TEMP_SCALE_CEL MCP9800_TEMP_SCALE_CEL
#define MCP980X_TEMP_ABS_MASK MCP9800_TEMP_ABS_MASK
#define MCP980X_TEMP_SIGN_BIT MCP9800_TEMP_SIGN_BIT

#endif /* ZEPHYR_DRIVERS_SENSOR_MCP980X_MCP9800_H_ */
