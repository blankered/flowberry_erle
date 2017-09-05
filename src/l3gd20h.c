#include "l3gd20h.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#define GYRO_AUTO_INCREMENT	(1 << 7)

#define GYRO_REG_CTRL1		0x20
#define GYRO_REG_CTRL4		0x23
#define GYRO_REG_CTRL5		0x24
#define GYRO_REG_OUT_TEMP	0x26
#define GYRO_REG_STATUS		0x27
#define GYRO_REG_OUT_X_L	0x28
#define GYRO_REG_LOW_ODR	0x39
#define GYRO_REG_WHO_AM_I	0x0f

#define GYRO_BIT_CTRL1_DR1		(1 << 7)
#define GYRO_BIT_CTRL1_DR0		(1 << 6)
#define GYRO_BIT_CTRL1_BW1		(1 << 5)
#define GYRO_BIT_CTRL1_BW0		(1 << 4)
#define GYRO_BIT_CTRL1_PD		(1 << 3)
#define GYRO_BIT_CTRL1_ZEN		(1 << 2)
#define GYRO_BIT_CTRL1_XEN		(1 << 1)
#define GYRO_BIT_CTRL1_YEN		(1 << 0)
#define GYRO_BIT_CTRL4_FS1		(1 << 5)
#define GYRO_BIT_CTRL4_FS0		(1 << 4)
#define GYRO_BIT_CTRL5_OUT_SEL1		(1 << 1)
#define GYRO_BIT_CTRL5_HPEN		(1 << 4)
#define GYRO_BIT_STATUS_ZYXOR		(1 << 7)
#define GYRO_BIT_LOW_ODR_SW_RES		(1 << 2)
#define GYRO_BIT_LOW_ODR_LOW_ODR	(1 << 0)

static int m_fd;
static float m_sensitivity;

static bool l3gd20h_write_reg(uint8_t reg, uint8_t value);
static bool l3gd20h_read_reg(uint8_t reg, uint8_t *p_value);
static bool l3gd20h_read_block(uint8_t reg, uint8_t *p_buffer, uint8_t length);

static bool l3gd20h_write_reg(uint8_t reg, uint8_t value)
{
	L3GD20H_DEBUG("[L3GD20H_DEBUG] Writing value 0x%02x to reg 0x%02x.\n",
		      value, reg);

	int ret = i2c_smbus_write_byte_data(m_fd, reg, value);
	if (ret == -1) {
		fprintf(stderr, "[L3GD20H] Failed to write register %d.\n", reg);
		return false;
	}

	return true;
}

static bool l3gd20h_read_reg(uint8_t reg, uint8_t *p_value)
{
	L3GD20H_DEBUG("[L3GD20H_DEBUG] Reading byte from reg 0x%02x.\n",
		      reg);

	int ret = i2c_smbus_read_i2c_block_data(m_fd, reg, 1, p_value);
	if (ret == -1) {
		fprintf(stderr, "[L3GD20H] Failed to read register %d.\n", reg);
		return false;
	}

	return true;
}

static bool l3gd20h_read_block(uint8_t reg, uint8_t *p_buffer, uint8_t length)
{
	int ret = i2c_smbus_read_i2c_block_data(m_fd, reg, length, p_buffer);
	if (ret == -1) {
		fprintf(stderr, "[L3GD20H] Failed to read register %d.\n", reg);
		return false;
	}

	return true;
}

bool l3gd20h_init(char *p_path, uint8_t i2c_addr, l3gd20h_init_t *p_init)
{
	uint8_t ctrl1_reg_value = 0;
	uint8_t ctrl4_reg_value = 0;
	uint8_t low_odr_reg_value = 0;
	uint8_t reg_value = 0;
	m_sensitivity = 0.0f;

	switch (p_init->range) {
	case L3GD20H_RANGE_245_DPS:
		m_sensitivity = 8.75f;
		break;
	case L3GD20H_RANGE_500_DPS:
		ctrl4_reg_value |= GYRO_BIT_CTRL4_FS0;
		m_sensitivity = 17.5f;
		break;
	case L3GD20H_RANGE_2000_DPS:
		ctrl4_reg_value |= GYRO_BIT_CTRL4_FS1;
		m_sensitivity = 70.0f;
		break;
	default:
		perror("[L3GD20H] Unknown l3gd20h_range_t.\n");
		return false;
	}

	switch (p_init->data_rate) {
	case L3GD20H_DATA_RATE_12_5HZ_NO_CUTOFF:
		low_odr_reg_value |= GYRO_BIT_LOW_ODR_LOW_ODR;
		break;
	case L3GD20H_DATA_RATE_25HZ_NO_CUTOFF:
		low_odr_reg_value |= GYRO_BIT_LOW_ODR_LOW_ODR;
		ctrl1_reg_value |= GYRO_BIT_CTRL1_DR0;
		break;
	case L3GD20H_DATA_RATE_50HZ_CUTOFF_16_6HZ:
		low_odr_reg_value |= GYRO_BIT_LOW_ODR_LOW_ODR;
		ctrl1_reg_value |= GYRO_BIT_CTRL1_DR1;
		break;
	case L3GD20H_DATA_RATE_100HZ_CUTOFF_12_5HZ:
		break;
	case L3GD20H_DATA_RATE_100HZ_CUTOFF_25HZ:
		ctrl1_reg_value |= GYRO_BIT_CTRL1_BW0;
		break;
	case L3GD20H_DATA_RATE_200HZ_CUTOFF_12_5HZ:
		ctrl1_reg_value |= GYRO_BIT_CTRL1_DR0;
		break;
	case L3GD20H_DATA_RATE_200HZ_CUTOFF_70HZ:
		ctrl1_reg_value |= (GYRO_BIT_CTRL1_DR0 | GYRO_BIT_CTRL1_BW1 |
				    GYRO_BIT_CTRL1_BW0);
		break;
	case L3GD20H_DATA_RATE_400HZ_CUTOFF_20HZ:
		ctrl1_reg_value |= GYRO_BIT_CTRL1_DR1;
		break;
	case L3GD20H_DATA_RATE_400HZ_CUTOFF_25HZ:
		ctrl1_reg_value |= (GYRO_BIT_CTRL1_DR1 | GYRO_BIT_CTRL1_BW0);
		break;
	case L3GD20H_DATA_RATE_400HZ_CUTOFF_50HZ:
		ctrl1_reg_value |= (GYRO_BIT_CTRL1_DR1 | GYRO_BIT_CTRL1_BW1);
		break;
	case L3GD20H_DATA_RATE_400HZ_CUTOFF_110HZ:
		ctrl1_reg_value |= (GYRO_BIT_CTRL1_DR1 | GYRO_BIT_CTRL1_BW1 |
				    GYRO_BIT_CTRL1_BW0);
		break;
	case L3GD20H_DATA_RATE_800HZ_CUTOFF_30HZ:
		ctrl1_reg_value |= (GYRO_BIT_CTRL1_DR1 | GYRO_BIT_CTRL1_DR0);
		break;
	case L3GD20H_DATA_RATE_800HZ_CUTOFF_35HZ:
		ctrl1_reg_value |= (GYRO_BIT_CTRL1_DR1 | GYRO_BIT_CTRL1_DR0 |
				    GYRO_BIT_CTRL1_BW0);
		break;
	case L3GD20H_DATA_RATE_800HZ_CUTOFF_100HZ:
		ctrl1_reg_value |= (GYRO_BIT_CTRL1_DR1 | GYRO_BIT_CTRL1_DR0 |
				    GYRO_BIT_CTRL1_BW1 | GYRO_BIT_CTRL1_BW0);
		break;
	default:
		perror("[L3GD20H] Unknown l3gd20h_data_rate_t.\n");
		return false;
	}

	/* Open bus */
	if ((m_fd = open(p_path, O_RDWR)) == -1) {
		fprintf(stderr, "[L3GD20H] Can't open I2C bus %s.\n", p_path);
		return false;
	}

	if (ioctl(m_fd, I2C_SLAVE, i2c_addr) == -1) {
		perror("[L3GD20H] Can't find I2C device.\n");
		return false;
	}

	/* Test chip ID */
	l3gd20h_read_reg(GYRO_REG_WHO_AM_I, &reg_value);

	if (reg_value != 0b11010111) {
		fprintf(stderr, "[L3GD20H] WHO_AM_I value %x is invalid.\n",
				reg_value);
		return false;
	}

	/* Reset chip and wait until it gets ready */
	reg_value = GYRO_BIT_LOW_ODR_SW_RES;
	l3gd20h_write_reg(GYRO_REG_LOW_ODR, reg_value);

	int reset_timeout = 0;
	while ((reg_value & GYRO_BIT_LOW_ODR_SW_RES) != 0) {
		usleep(1000);
		if (!l3gd20h_read_reg(GYRO_REG_LOW_ODR, &reg_value))
			return false;

		if (reset_timeout++ > 100) {
			perror("[L3GD20H] Reset timeout: 100 ms.\n");
			return false;
		}
	}

	/* Configure range */
	if (!l3gd20h_write_reg(GYRO_REG_LOW_ODR, low_odr_reg_value))
		return false;

	if (!l3gd20h_write_reg(GYRO_REG_CTRL4, ctrl4_reg_value))
		return false;

	if (p_init->enable_lowpass || p_init->enable_highpass) {
		uint8_t val = 0;

		if (p_init->enable_lowpass)
			val |= GYRO_BIT_CTRL5_OUT_SEL1;

		if (p_init->enable_highpass)
			val |= GYRO_BIT_CTRL5_HPEN;

		if (!l3gd20h_write_reg(GYRO_REG_CTRL5, val))
			return false;
	}

	/* Enable X, Y and Z axes and enter the Normal Mode */
	ctrl1_reg_value |= (GYRO_BIT_CTRL1_XEN | GYRO_BIT_CTRL1_YEN |
			    GYRO_BIT_CTRL1_ZEN | GYRO_BIT_CTRL1_PD );
	if (!l3gd20h_write_reg(GYRO_REG_CTRL1, ctrl1_reg_value))
		return false;

	return true;
}

bool l3gd20h_read(l3gd20h_data_t *p_data, bool read_overrun_and_temperature)
{
	uint8_t buffer[8];
	bool res;
	int16_t x, y, z;

	if (read_overrun_and_temperature)
		res = l3gd20h_read_block((GYRO_REG_OUT_TEMP |
					 GYRO_AUTO_INCREMENT), buffer, 8);
	else
		res = l3gd20h_read_block((GYRO_REG_OUT_X_L |
					 GYRO_AUTO_INCREMENT), buffer, 6);

	if (!res)
		return false;

	if (read_overrun_and_temperature) {
		p_data->temperature = 25 + (int8_t)buffer[0];
		
		if ((buffer[1] & GYRO_BIT_STATUS_ZYXOR) != 0)
			p_data->overrun = true;
		else
			p_data->overrun = false;

		x = (int16_t)(buffer[2] | (buffer[3] << 8));
		y = (int16_t)(buffer[4] | (buffer[5] << 8));
		z = (int16_t)(buffer[6] | (buffer[7] << 8));
	} else {
		p_data->temperature = 0;
		p_data->overrun = false;
		x = (int16_t)(buffer[0] | (buffer[1] << 8));
		y = (int16_t)(buffer[2] | (buffer[3] << 8));
		z = (int16_t)(buffer[4] | (buffer[5] << 8));
	}

	p_data->rate_x = (float)x * m_sensitivity / 1000;
	p_data->rate_y = (float)y * m_sensitivity / 1000;
	p_data->rate_z = (float)z * m_sensitivity / 1000;

	return true;
}

bool l3gd20h_read_temperature(int8_t *p_temperature)
{
	uint8_t reg_value;

	if (!l3gd20h_read_reg(GYRO_REG_OUT_TEMP, &reg_value))
		return false;

	*p_temperature = 25 + (int8_t)reg_value;

	return true;
}

bool l3gd20h_close(void)
{
	/* Enter the Power Down mode */
	if (!l3gd20h_write_reg(GYRO_REG_CTRL1, 0))
		return false;

	/* Close bus */
	if (close(m_fd) == -1) {
		perror("[L3GD20H] Failed to close I2C bus.\n");
		return false;
	}

	return true;
}
