#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include "l3gd20h.h"

#define GYRO_CALIB_PATH		"gyro_calib.txt"

#define GYRO_DEFAULT_I2C_ADDRESS	0x6b
#define GYRO_READOUT_PERIOD_MS		10	/* 100 Hz */
#define GYRO_CALIB_TIME_SEC		60

typedef struct {
	float x;
	float y;
	float z;
} gyro_angle_t;

volatile sig_atomic_t m_run = true;

static suseconds_t microseconds();
static void fix_angle_range(float *p_angle);
static void gyro_integrate(gyro_angle_t *p_angle, l3gd20h_data_t *p_data, float period);
static bool gyro_load_calibration(char *p_filename, l3gd20h_data_t *p_calib);
static bool gyro_calibrate(char *p_filename);

void sigint_handler(int signum)
{
    m_run = false;
}

static suseconds_t microseconds()
{
	static struct timeval tv;
	gettimeofday(&tv, NULL);

	return ((tv.tv_sec * 1000000UL) + tv.tv_usec);
}

static void fix_angle_range(float *p_angle)
{
	while (*p_angle < -180)
		*p_angle += 360;

	while (*p_angle > 180)
		*p_angle -= 360; 
}

static void gyro_integrate(gyro_angle_t *p_angle, l3gd20h_data_t *p_data, float period)
{
	p_angle->x += p_data->rate_x * period;
	p_angle->y += p_data->rate_y * period;
	p_angle->z += p_data->rate_z * period;

	fix_angle_range(&p_angle->x);
	fix_angle_range(&p_angle->y);
	fix_angle_range(&p_angle->z);
}

static bool gyro_load_calibration(char *p_filename, l3gd20h_data_t *p_calib)
{
	FILE *fp = fopen(p_filename, "r");

	if (fp == NULL)
		return false;

	int ret;
	int samples = 0;

	double cx = 0;
	double cy = 0;
	double cz = 0;
	int ct = 0;

	while (1) {
		float x, y, z;
		int t;
		ret = fscanf(fp, "%e %e %e %d", &x, &y, &z, &t);
		if (ret == 4) {
			samples++;
			cx += x;
			cy += y;
			cz += z;
			ct += t;
		} else {
			break;
		}
	}

	if (samples > 0) {
		p_calib->rate_x = (float)(cx/samples);
		p_calib->rate_y = (float)(cy/samples);
		p_calib->rate_z = (float)(cz/samples);
		p_calib->temperature = ct/samples;
	} else {
		memset(p_calib, 0, sizeof(l3gd20h_data_t));
	}

	printf("Calibration: read %d samples from %s.\n", samples, p_filename);
	printf("Calibration: %f, %f, %f (%d °C)\n", p_calib->rate_x,
	       p_calib->rate_y, p_calib->rate_z, p_calib->temperature);

	fclose(fp);

	return true;
}

static bool gyro_calibrate(char *p_filename)
{
	l3gd20h_data_t gyro_data;
	memset(&gyro_data, 0, sizeof(gyro_data));

	int samples = 0;
	int overruns = 0;

	FILE *fp = fopen(p_filename, "w");

	if (fp == NULL)
		return false;

	suseconds_t time_start = microseconds();

	while (m_run && (microseconds()-time_start) < GYRO_CALIB_TIME_SEC*1000000UL) {
		suseconds_t t1 = microseconds();

		if (!l3gd20h_read(&gyro_data, true)) {
			printf("Can't read gyro!\n");
			l3gd20h_close();
			exit(EXIT_FAILURE);
		}

		samples++;

		if (gyro_data.overrun && samples > 1)
			overruns++;

		fprintf(fp, "%e %e %e %d\n", gyro_data.rate_x, gyro_data.rate_y,
			gyro_data.rate_z, gyro_data.temperature);

		printf("\rFinished %d samples (%d overruns)  ", samples, overruns);
		fflush(stdout);

		suseconds_t t2 = microseconds();

		if (t2 > t1+(GYRO_READOUT_PERIOD_MS*1000UL))
			usleep((GYRO_READOUT_PERIOD_MS*1000UL) - (t2-t1));
		else
			usleep(100);
	}

	printf("\n");

	fclose(fp);

	return true;
}

int main(int argc, char **argv)
{
	signal(SIGINT, sigint_handler);

	uint8_t i2c_dev_addr = GYRO_DEFAULT_I2C_ADDRESS;
	char *i2c_bus_path = NULL;
	char calib_file_path[] = GYRO_CALIB_PATH;
	bool do_calib = false;
	bool single_line = false;

	l3gd20h_init_t gyro_init;
	l3gd20h_data_t gyro_data;
	l3gd20h_data_t gyro_calib;
	gyro_angle_t gyro_angle;
	memset(&gyro_init, 0, sizeof(gyro_init));
	memset(&gyro_data, 0, sizeof(gyro_data));
	memset(&gyro_calib, 0, sizeof(gyro_calib));
	memset(&gyro_angle, 0, sizeof(gyro_angle));

	/* Handle -options */
	int c;
	while ((c = getopt(argc, argv, "a:chs")) != -1) {
		switch (c) {
		case 'a':
			sscanf(optarg, "%x", &i2c_dev_addr);
			break;
		case 'c':
			do_calib = true;
			break;
		case 's':
			single_line = true;
			break;
		case 'h':
			printf("Usage: %s [-a <ADDR>] [-c] [-s] I2C_BUS\n", argv[0]);
			printf("Reads angle rates from the L3GD20H gyroscope and integrates it to angles.\n");
			printf("Please provide calibration data in file %s. Otherwise zeros will be used.\n", calib_file_path);
			printf("\n");
			printf("  -a ADDR  Sets I2C device address (Hex number, default 0x%02x).\n", i2c_dev_addr);
			printf("  -c       Perofms calibration procedure and stores it into file %s.\n", calib_file_path);
			printf("  -s       Single line option (nicer output for humans).\n");
			printf("  -h       Displays this help.\n");
			return 0;
			break;
		case '?':
			if (optopt == 'a')
				fprintf(stderr, "Option -%c requires an argument.\nUse %s -h for help.\n", optopt, argv[0]);
			else
				fprintf(stderr, "Unknown option character '%c'.\nUse %s -h for help.\n", optopt, argv[0]);
			return 1;
		default:
			fprintf(stderr, "Usage: %s [-a <ADDR>] [-c] [-s] I2C_BUS\n", argv[0]);
			return 1;
		}
	}

	/* Save I2C bus path */
	if (optind < argc) {
		i2c_bus_path = argv[optind];
	} else {
		fprintf(stderr, "I2C bus path must be specified.\nUse %s -h for help.\n", argv[0]);
		return 1;
	}

	gyro_init.range = L3GD20H_RANGE_245_DPS;
	gyro_init.data_rate = L3GD20H_DATA_RATE_100HZ_CUTOFF_25HZ;
	gyro_init.enable_highpass = false;
	gyro_init.enable_lowpass = true;

	if (!l3gd20h_init(i2c_bus_path, i2c_dev_addr, &gyro_init)) {
		fprintf(stderr, "Can't open device 0x%02x or bus %s.\n", i2c_dev_addr, i2c_bus_path);
		exit(EXIT_FAILURE);
	}

	usleep(500000UL);

	if (do_calib) {
		printf("Starting calibration procedure. Please do not move the device.\n");
		if (!gyro_calibrate(calib_file_path)) {
			fprintf(stderr, "Can't save calibration data to %s.\n", calib_file_path);
			return 1;
		}

		printf("Calibration data was successfully written to %s.\n", calib_file_path);

		if (!l3gd20h_close())
			exit(EXIT_FAILURE);

		return (EXIT_SUCCESS);
	} else {
		if (!gyro_load_calibration(calib_file_path, &gyro_calib)) {
			printf("Unable to read calibration data from %s. Using default values.\n", calib_file_path);
		}
	}

	float dt = (float)GYRO_READOUT_PERIOD_MS/1000.0f;

	while (m_run) {
		suseconds_t time_start = microseconds();

		if (!l3gd20h_read(&gyro_data, true)) {
			printf("\n");
			l3gd20h_close();
			exit(EXIT_FAILURE);
		}

		gyro_data.rate_x -= gyro_calib.rate_x;
		gyro_data.rate_y -= gyro_calib.rate_y;
		gyro_data.rate_z -= gyro_calib.rate_z;

		gyro_integrate(&gyro_angle, &gyro_data, dt);

		if (single_line) {
			printf("\rX: %.1f, Y: %.1f, Z: %.1f (%d °C, overrun: %d)     ",
			       gyro_angle.x, gyro_angle.y, gyro_angle.z,
			       gyro_data.temperature, gyro_data.overrun);
			fflush(stdout);
		} else {
			printf("%f %f %f %d\n", gyro_angle.x, gyro_angle.y,
			       gyro_angle.z, gyro_data.temperature);
			fflush(stdout);
		}

		suseconds_t time_end = microseconds();

		if (time_end > time_start+(GYRO_READOUT_PERIOD_MS*1000UL))
			usleep((GYRO_READOUT_PERIOD_MS*1000UL) - (time_end-time_start));
		else
			usleep(100);

#if 0
		if (gyro_data.overrun)  {
			printf("\n");
			perror("Overrun was detected!\n");
			l3gd20h_close();
			exit(EXIT_FAILURE);
		}
#endif
	}

	printf("\n");
	if (!l3gd20h_close())
		exit(EXIT_FAILURE);

	return (EXIT_SUCCESS);
}
