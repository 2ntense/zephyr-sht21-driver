#ifndef ZEPHYR_DRIVERS_SENSOR_SHT21_SHT21_H_
#define ZEPHYR_DRIVERS_SENSOR_SHT21_SHT21_H_

#include <device.h>
#include <kernel.h>
#include <drivers/gpio.h>

#define SHT21_CMD_TRIG_T_MEAS_HM	0b11100011
#define SHT21_CMD_TRIG_RH_MEAS_HM	0b11100101
#define SHT21_CMD_TRIG_T_MEAS_NHM	0b11110011
#define SHT21_CMD_TRIG_RH_MEAS_NHM	0b11110101
#define SHT21_CMD_USER_REG_WRITE	0b11100110
#define SHT21_CMD_USER_REG_READ		0b11100111
#define SHT21_CMD_SOFT_RESET		0b11111110

struct sht21_config {
	char *bus_name;
	uint8_t base_address;
};

struct sht21_data {
	const struct device *dev;
	const struct device *bus;

	uint16_t t_sample;
	uint16_t rh_sample;

};

static inline uint8_t sht21_i2c_address(const struct device *dev)
{
	const struct sht21_config *dcp = dev->config;

	return dcp->base_address;
}

static inline const struct device *sht21_i2c_device(const struct device *dev)
{
	const struct sht21_data *ddp = dev->data;

	return ddp->bus;
}

#endif /* ZEPHYR_DRIVERS_SENSOR_SHT21_SHT21_H_ */
