#define DT_DRV_COMPAT sensirion_sht21

#include <device.h>
#include <drivers/i2c.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <logging/log.h>

#include "sht21.h"

LOG_MODULE_REGISTER(SHT21, CONFIG_SENSOR_LOG_LEVEL);

static uint8_t sht21_calc_crc(const uint16_t value)
{
	const uint16_t poly = 0x131; //P(x)=x^8+x^5+x^4+1 = 100110001
	uint8_t buf[2] = {value >> 8, value & 0xff};
	uint8_t crc = 0;

	for (uint8_t byte = 0; byte < sizeof(buf); ++byte)
	{ 
		crc ^= buf[byte];
		for (uint8_t bit = 8; bit > 0; --bit) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ poly;
			}
			else {
				crc = (crc << 1);
			}
		}
	}
	return crc;
}

static int sht21_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	struct sht21_data *data = dev->data;
	const struct device *i2c = sht21_i2c_device(dev);
	uint8_t address = sht21_i2c_address(dev);
	uint8_t tx_buf;
	uint8_t rx_buf[3];
	uint16_t t_sample, rh_sample;

	switch (chan)
	{
	case SENSOR_CHAN_AMBIENT_TEMP:
		tx_buf = SHT21_CMD_TRIG_T_MEAS_HM;
		if (i2c_write_read(i2c, address, &tx_buf, sizeof(tx_buf), rx_buf, sizeof(rx_buf)) < 0) {
			LOG_DBG("Failed to read data sample!");
			return -EIO;
		}
		t_sample = (rx_buf[0] << 8) | rx_buf[1];
		if(sht21_calc_crc(t_sample) != rx_buf[2]) {
			LOG_DBG("Received invalid temperature CRC!");
			return -EIO;
		}
		data->t_sample = t_sample;
		break;
	case SENSOR_CHAN_HUMIDITY:
		tx_buf = SHT21_CMD_TRIG_RH_MEAS_HM;
		if (i2c_write_read(i2c, address, &tx_buf, sizeof(tx_buf), rx_buf, sizeof(rx_buf)) < 0) {
			LOG_DBG("Failed to read data sample!");
			return -EIO;
		}
		rh_sample = (rx_buf[0] << 8) | rx_buf[1];
		if(sht21_calc_crc(rh_sample) != rx_buf[2]) {
			LOG_DBG("Received invalid temperature CRC!");
			return -EIO;
		}
		data->rh_sample = rh_sample;
		break;
	default:
		break;
	}

	return 0;
}

static int sht21_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	const struct sht21_data *data = dev->data;
	uint64_t tmp;

	if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
		tmp = ((data->t_sample * 17572) >> 16) - 4685;
		val->val1 = (int32_t)(tmp / 100);
		val->val2 = (int32_t)(tmp % 100) * 10000;
	} else if (chan == SENSOR_CHAN_HUMIDITY) {
		tmp = ((data->rh_sample * 12500) >> 16) - 600;
		val->val1 = (int32_t)(tmp / 100);
		val->val2 = (int32_t)(tmp % 100) * 10000;
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api sht21_driver_api = {
	.sample_fetch = sht21_sample_fetch,
	.channel_get = sht21_channel_get,
};

static int sht21_init(const struct device *dev)
{
	struct sht21_data *data = dev->data;
	const struct sht21_config *cfg = dev->config;
	const struct device *i2c = device_get_binding(cfg->bus_name);

	if (i2c == NULL) {
		LOG_DBG("Failed to get pointer to %s device!",
			cfg->bus_name);
		return -EINVAL;
	}
	data->bus = i2c;

	if (!cfg->base_address) {
		LOG_DBG("No I2C address");
		return -EINVAL;
	}
	data->dev = dev;

	return 0;
}

struct sht21_data sht21_driver;
static const struct sht21_config sht21_cfg = {
	.bus_name = DT_INST_BUS_LABEL(0),
	.base_address = DT_INST_REG_ADDR(0),
};

DEVICE_DT_INST_DEFINE(0, sht21_init, device_pm_control_nop,
		    &sht21_driver, &sht21_cfg,
		    POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		    &sht21_driver_api);
