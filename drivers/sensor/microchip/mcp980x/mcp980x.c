/*
 * Copyright (c) 2019 Peter Bigot Consulting, LLC
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "mcp980x.h"

#if CONFIG_MCP9808
#define DT_DRV_COMPAT microchip_mcp9808
#elif CONFIG_MCP9800
#define DT_DRV_COMPAT microchip_mcp9800
#else
#error "Unsupported MCP chipset"
#endif

LOG_MODULE_REGISTER(MCP980X, CONFIG_SENSOR_LOG_LEVEL);

int mcp980x_reg_read_16bit(const struct device *dev, uint8_t reg, uint16_t *val)
{
	const struct mcp980x_config *cfg = dev->config;
	int rc = i2c_write_read_dt(&cfg->i2c, &reg, sizeof(reg), val, sizeof(*val));

	if (rc == 0) {
		*val = sys_be16_to_cpu(*val);
	}

	return rc;
}

int mcp9800_reg_read_8bit(const struct device *dev, uint8_t reg, uint8_t *val)
{
	const struct mcp980x_config *cfg = dev->config;

	return i2c_write_read_dt(&cfg->i2c, &reg, sizeof(reg), val, sizeof(*val));
}

int mcp980x_reg_write_16bit(const struct device *dev, uint8_t reg, uint16_t val)
{
	const struct mcp980x_config *cfg = dev->config;

	uint8_t buf[3];

	buf[0] = reg;
	sys_put_be16(val, &buf[1]);

	return i2c_write_dt(&cfg->i2c, buf, sizeof(buf));
}

int mcp980x_reg_write_8bit(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct mcp980x_config *cfg = dev->config;
	uint8_t buf[2] = {
		reg,
		val,
	};

	return i2c_write_dt(&cfg->i2c, buf, sizeof(buf));
}

static int mcp980x_set_temperature_resolution(const struct device *dev, uint8_t resolution)
{
#if CONFIG_MCP9800
	return mcp980x_reg_write_8bit(dev, MCP9800_REG_CONFIG, resolution << 5);
#elif CONFIG_MCP9808
	return mcp980x_reg_write_8bit(dev, MCP9808_REG_RESOLUTION, resolution);
#endif
}

static int mcp980x_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct mcp980x_data *data = dev->data;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_AMBIENT_TEMP);

	return mcp980x_reg_read_16bit(dev, MCP980X_REG_TEMP_AMB, &data->reg_val);
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
	int rc = 0;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	rc = mcp980x_set_temperature_resolution(dev, cfg->resolution);
	if (rc) {
		LOG_ERR("Could not set the resolution of mcp980x module");
		return rc;
	}

#ifdef CONFIG_MCP980X_TRIGGER
	if (cfg->int_gpio.port) {
		rc = mcp980x_setup_interrupt(dev);
	}
#endif /* CONFIG_MCP980X_TRIGGER */

	return rc;
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
