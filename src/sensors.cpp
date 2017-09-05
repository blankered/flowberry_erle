#include "sensors.h"
#include <unistd.h>
#include <pthread.h>
#include "l3gd20h.h"
#include "sonar.h"

#define I2C_BUS_PATH		"/dev/i2c-1"

#define GYRO_DEFAULT_I2C_ADDRESS	0x6b
#define GYRO_CALIB_PATH			"gyro_calib.txt"
#define GYRO_READOUT_PERIOD_MS		10

#define SONAR_PORT_PATH			"/dev/ttyAMA0"
#define SONAR_READOUT_PERIOD_MS		100

using namespace cv;
using namespace std;

static bool m_initialized;

static struct {
	pthread_t thread;
	pthread_mutex_t mutex;
	bool run;

	l3gd20h_data_t calib;

	struct {
		double x;
		double y;
		double z;
		int temperature;
		int overruns;
	} acc_angle;
} m_gyro;

static struct {
	pthread_t thread;
	pthread_mutex_t mutex;
	bool run;

	int distance_mm;
} m_sonar;

static bool m_enable_sonar;

static void *gyro_thread(void *ptr)
{
	suseconds_t t1, t2;
	l3gd20h_data_t gyro_data;

	memset(&gyro_data, 0, sizeof(gyro_data));

	t2 = microseconds();

	while (m_gyro.run) {
		t1 = microseconds();

		if (!l3gd20h_read(&gyro_data, true)) {
			fprintf(stderr, "[sensors] Can't read gyro!\n");
			memset(&gyro_data, 0, sizeof(gyro_data));
		}

		gyro_data.rate_x -= m_gyro.calib.rate_x;
		gyro_data.rate_y -= m_gyro.calib.rate_y;
		gyro_data.rate_z -= m_gyro.calib.rate_z;

		pthread_mutex_lock(&m_gyro.mutex);

		m_gyro.acc_angle.x = gyro_data.rate_x;
		m_gyro.acc_angle.y = gyro_data.rate_y;
		m_gyro.acc_angle.z = gyro_data.rate_z;
		m_gyro.acc_angle.temperature = gyro_data.temperature;

		if (gyro_data.overrun)
			m_gyro.acc_angle.overruns++;

		pthread_mutex_unlock(&m_gyro.mutex);

		t2 = microseconds();

		if ((t2-t1) < (GYRO_READOUT_PERIOD_MS*1000UL))
			usleep(GYRO_READOUT_PERIOD_MS*1000UL - (t2-t1));
		else
			usleep(100);
	}

	l3gd20h_close();
}

static void *sonar_thread(void *ptr)
{
	suseconds_t t1, t2;
	int dist;

	while (m_sonar.run) {
		t1 = microseconds();

		if (sonar_read(&dist)) {
			pthread_mutex_lock(&m_sonar.mutex);
			m_sonar.distance_mm = dist;
			pthread_mutex_unlock(&m_sonar.mutex);

			t2 = microseconds();
			if ((t2-t1) < (SONAR_READOUT_PERIOD_MS*1000UL))
				usleep(SONAR_READOUT_PERIOD_MS*1000UL - (t2-t1));
			else
				usleep(100);
		}
	}

	sonar_close();
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

bool sensors_init(bool enable_sonar)
{
	DBG("sensors_init()");
	char *i2c_path = (char *)I2C_BUS_PATH;
	char *sonar_path = (char *)SONAR_PORT_PATH;

	m_enable_sonar = enable_sonar;

	l3gd20h_init_t gyro_init;
	memset(&gyro_init, 0, sizeof(gyro_init));

	gyro_init.range = L3GD20H_RANGE_245_DPS;
	gyro_init.data_rate = L3GD20H_DATA_RATE_100HZ_CUTOFF_25HZ;
	gyro_init.enable_highpass = false;
	gyro_init.enable_lowpass = true;

	if (!l3gd20h_init(i2c_path, GYRO_DEFAULT_I2C_ADDRESS, &gyro_init)) {
		fprintf(stderr, "[sensors] Can't open gyro addr 0x%02x or bus %s.\n",
			GYRO_DEFAULT_I2C_ADDRESS, i2c_path);
		return false;
	}

	if (m_enable_sonar) {
		if (!sonar_init(sonar_path)) {
			fprintf(stderr, "[sensors] Can't open sonar port %s.\n",
				sonar_path);
			return false;
		}
	}

	if (!gyro_load_calibration((char*)GYRO_CALIB_PATH, &m_gyro.calib)) {
		fprintf(stderr, "[sensors] Can't load gyro calibration!\n");
		return false;
	}

	m_initialized = true;

	return true;
}

bool sensors_start(void)
{
	if (!m_initialized)
		return false;

	DBG("sensors_start()");

	m_gyro.run = true;
	m_sonar.run = true;

	pthread_mutex_init(&m_gyro.mutex, NULL);
	pthread_mutex_init(&m_sonar.mutex, NULL);

	int rc = pthread_create(&m_gyro.thread, NULL, gyro_thread, NULL);
	if (rc) {
		ERR("Unable to create gyro thread: " << rc);
		return false;
	}

	if (m_enable_sonar) {
		rc = pthread_create(&m_sonar.thread, NULL, sonar_thread, NULL);
		if (rc) {
			ERR("Unable to create sonar thread: " << rc);
			return false;
		}
	}

	return true;
}

void sensors_read(sensors_data_t *p_data)
{
	DBG("sensors_read():");

	pthread_mutex_lock(&m_gyro.mutex);

	DBG("gyro: " << m_gyro.acc_angle.x << ", " << m_gyro.acc_angle.y << ", "
	    << m_gyro.acc_angle.z << " (" << m_gyro.acc_angle.temperature
	    << " deg C, " << m_gyro.acc_angle.overruns << " overruns)");

	p_data->gyro.x = m_gyro.acc_angle.x;
	p_data->gyro.y = m_gyro.acc_angle.y;
	p_data->gyro.z = m_gyro.acc_angle.z;
	p_data->gyro.temperature = m_gyro.acc_angle.temperature;

	m_gyro.acc_angle.overruns = 0;
	//memset(&m_gyro.acc_angle, 0, sizeof(m_gyro.acc_angle));

	pthread_mutex_unlock(&m_gyro.mutex);

	pthread_mutex_lock(&m_sonar.mutex);

	DBG("sonar: " << m_sonar.distance_mm << " mm");
	p_data->sonar.distance_mm = m_sonar.distance_mm;

	pthread_mutex_unlock(&m_sonar.mutex);	
}

void sensors_stop(void)
{
	if (!m_initialized)
		return;

	DBG("sensors_stop()");

	m_gyro.run = false;
	m_sonar.run = false;

	//pthread_destroy(&m_gyro.thread);
	//pthread_destroy(&m_sonar.thread);
	pthread_mutex_destroy(&m_gyro.mutex);
	pthread_mutex_destroy(&m_sonar.mutex);
}

void sensors_compensate(vector<Point2f>& pts_src, vector<Point2f>& pts_dst)
{
	suseconds_t t1, t2;
	static suseconds_t prev_t;

	t1 = microseconds();

	double corr_x = 0;
	double corr_y = 0;

	if (prev_t != 0) {
		double dt = (t1-prev_t)/1000000.0;

		pthread_mutex_lock(&m_gyro.mutex);
		double omega_x = M_PI * m_gyro.acc_angle.x / 180.0;
		double omega_y = M_PI * m_gyro.acc_angle.y / 180.0;
		pthread_mutex_unlock(&m_gyro.mutex);

		/* f / (b*s) = 531.9335 */
		corr_x = 531.9335 * tan(omega_x * dt);
		corr_y = 531.9335 * tan(omega_y * dt);
	}

	int count = pts_src.size(); /* TODO: Use an iterator */
	for(int i = 0; i < count; i++) {
		pts_src[i].x -= corr_x;
		pts_src[i].y += corr_y;
	}

	t2 = microseconds();
	prev_t = t1;

	DBG("sensors_compensate(): [" << corr_x << ", " << corr_y << "], " << (t2-t2) << " us");
}
