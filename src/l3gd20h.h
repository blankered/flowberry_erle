#ifndef L3GD20H
#define L3GD20H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* #define L3GD20H_I2C_ADDRESS	0x6b */

#ifdef DEBUG
#define L3GD20H_DEBUG		printf
#else
#define L3GD20H_DEBUG(...)
#endif

typedef enum {
	L3GD20H_RANGE_245_DPS = 1,
	L3GD20H_RANGE_500_DPS,
	L3GD20H_RANGE_2000_DPS
} l3gd20h_range_t;

typedef enum {
	L3GD20H_DATA_RATE_12_5HZ_NO_CUTOFF = 1,
	L3GD20H_DATA_RATE_25HZ_NO_CUTOFF,
	L3GD20H_DATA_RATE_50HZ_CUTOFF_16_6HZ,
	L3GD20H_DATA_RATE_100HZ_CUTOFF_12_5HZ,
	L3GD20H_DATA_RATE_100HZ_CUTOFF_25HZ,
	L3GD20H_DATA_RATE_200HZ_CUTOFF_12_5HZ,
	L3GD20H_DATA_RATE_200HZ_CUTOFF_70HZ,
	L3GD20H_DATA_RATE_400HZ_CUTOFF_20HZ,
	L3GD20H_DATA_RATE_400HZ_CUTOFF_25HZ,
	L3GD20H_DATA_RATE_400HZ_CUTOFF_50HZ,
	L3GD20H_DATA_RATE_400HZ_CUTOFF_110HZ,
	L3GD20H_DATA_RATE_800HZ_CUTOFF_30HZ,
	L3GD20H_DATA_RATE_800HZ_CUTOFF_35HZ,
	L3GD20H_DATA_RATE_800HZ_CUTOFF_100HZ,
} l3gd20h_data_rate_t;

typedef struct {
	l3gd20h_range_t range;
	l3gd20h_data_rate_t data_rate;
	bool enable_lowpass;
	bool enable_highpass;
} l3gd20h_init_t;

typedef struct {
	float rate_x; /* Degrees */
	float rate_y; /* Degrees */
	float rate_z; /* Degrees */
	int temperature; /* Degrees of Celsius */
	bool overrun; /* Readout speed is insufficient */
} l3gd20h_data_t;

bool l3gd20h_init(char *p_path, uint8_t i2c_addr, l3gd20h_init_t *p_init);
bool l3gd20h_read(l3gd20h_data_t *p_data, bool read_overrun_and_temperature);
bool l3gd20h_read_temperature(int8_t *p_temperature);
bool l3gd20h_close(void);

#ifdef __cplusplus
}
#endif

#endif