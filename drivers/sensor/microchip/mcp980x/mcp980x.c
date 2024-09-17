/*
 * Copyright (c) 2024 Basalte bv
 * Copyright (c) 2019 Peter Bigot Consulting, LLC
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mcp980x

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include "mcp980x.h"

LOG_MODULE_REGISTER(MCP980X, CONFIG_SENSOR_LOG_LEVEL);

int mcp980x_reg_read(const struct device *dev, uint8_t reg, uint16_t *val, uint8_t len)
{
	const struct mcp980x_config *cfg = dev->config;
	int ret;

	ret = i2c_write_read_dt(&cfg->i2c, &reg, sizeof(reg), val, len);
	if (ret < 0) {
		return ret;
	}

	if (len == 2) {
		*val = sys_be16_to_cpu(*val);
	}

	return ret;
}

int mcp980x_reg_write(const struct device *dev, uint8_t reg, uint16_t val, uint8_t len)
{
	const struct mcp980x_config *cfg = dev->config;
	uint8_t buf[sizeof(reg) + sizeof(val)];

	if (len == 0 || len > 2) {
		LOG_ERR("Invalid len (%d)", len);
		return -EINVAL;
	}

	buf[0] = reg;

	if (len == 1) {
		buf[1] = val;
	} else {
		sys_put_be16(val, &buf[1]);
	}

	return i2c_write_dt(&cfg->i2c, buf, sizeof(reg) + len);
}

static int mcp980x_set_temperature_resolution(const struct device *dev, uint8_t resolution)
{

	int ret;

	uint16_t config = 0x00;
	ret = mcp980x_reg_read(dev, MCP980X_REG_CONFIG, &config, sizeof(uint8_t));
	if (ret < 0) {
		LOG_ERR("Failed to read config reg (%d)", ret);
		return ret;
	}

	config &= ~MCP980X_CFG_ADC_RESOLUTION;
	config |= ((resolution << 5) & MCP980X_CFG_ADC_RESOLUTION);

	return mcp980x_reg_write(dev, MCP980X_REG_CONFIG, config, sizeof(uint8_t));
}

static int mcp980x_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct mcp980x_data *data = dev->data;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_AMBIENT_TEMP);

	return mcp980x_reg_read(dev, MCP980X_REG_TEMP_AMB, &data->reg_val, sizeof(uint16_t));
}

static int mcp980x_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	const struct mcp980x_data *data = dev->data;
	int temp = mcp980x_temp_signed_from_reg(data->reg_val);

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_AMBIENT_TEMP);

	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	val->val1 = temp / MCP980X_TEMP_SCALE_CEL;
	temp -= val->val1 * MCP980X_TEMP_SCALE_CEL;
	val->val2 = (temp * 1000000) / MCP980X_TEMP_SCALE_CEL;

	return 0;
}

static const struct sensor_driver_api mcp980x_api_funcs = {
	.sample_fetch = mcp980x_sample_fetch,
	.channel_get = mcp980x_channel_get,
#ifdef CONFIG_MCP980X_TRIGGER
	.attr_set = mcp980x_attr_set,
	.trigger_set = mcp980x_trigger_set,
#endif /* CONFIG_MCP980X_TRIGGER */
};

int mcp980x_init(const struct device *dev)
{
	const struct mcp980x_config *cfg = dev->config;
	int ret;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	ret = mcp980x_set_temperature_resolution(dev, cfg->resolution);
	if (ret < 0) {
		LOG_ERR("Could not set the resolution of mcp980x module (%d)", ret);
		return ret;
	}

#ifdef CONFIG_MCP980X_TRIGGER
	if (cfg->int_gpio.port) {
		ret = mcp980x_setup_interrupt(dev);
	}
#endif /* CONFIG_MCP980X_TRIGGER */

	return ret;
}

#define MCP980X_DEFINE(inst)                                                                       \
	static struct mcp980x_data mcp980x_data_##inst;                                            \
                                                                                                   \
	static const struct mcp980x_config mcp980x_config_##inst = {                               \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.resolution = DT_INST_PROP(inst, resolution),                                      \
		IF_ENABLED(CONFIG_MCP980X_TRIGGER,                                                 \
			   (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}), ))};       \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, mcp980x_init, NULL, &mcp980x_data_##inst,               \
				     &mcp980x_config_##inst, POST_KERNEL,                          \
				     CONFIG_SENSOR_INIT_PRIORITY, &mcp980x_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(MCP980X_DEFINE)
