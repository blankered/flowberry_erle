#include "mavlog.h"

/* Inspired by example code at http://qgroundcontrol.org/dev/mavlink_linux_integration_tutorial */

#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "cv_queue.h"
#include "sensors.h"
#include "mavlink.h"

/* TODO: Make this configurable (14550 is the default port for QGroundControl) */
/* TODO: Allow hostname */
#define MAVLOG_TARGET_IP		"192.168.42.42"
#define MAVLOG_PORT			14550

/* TODO: Adjust */
#define MAVLOG_BUFFER_SIZE		1024
#define MAVLOG_HEARTBEAT_PERIOD_MS	1000

/* TODO: Determine correct values */
#define MAVLOG_SYSTEM_ID		1
#define MAVLOG_COMPONENT_ID		42
#define MAVLOG_SENSOR_ID		68

/* Conversion: */
#define PX2M		0.0019 /* (b*s)/f */

static pthread_t m_mavlink_thread;
static pthread_t m_heartbeat_thread;
static cv_queue<mavlink_message_t> m_msq_queue;
static volatile bool m_initialized;
static volatile bool m_run;

static void *mavlink_thread(void *ptr)
{
	int target_sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	struct sockaddr_in target_addr;
	uint8_t buffer[MAVLOG_BUFFER_SIZE];
	mavlink_message_t msg;
  
	memset(&target_addr, 0, sizeof(target_addr));
	target_addr.sin_family = AF_INET;
	target_addr.sin_addr.s_addr = inet_addr(MAVLOG_TARGET_IP);
	target_addr.sin_port = htons(MAVLOG_PORT);

	DBG("mavlog_thread(): Sending MavLink packets to " <<
	    MAVLOG_TARGET_IP << ":" << MAVLOG_PORT);

	while (m_run) {
		mavlink_message_t msg = m_msq_queue.remove();
		int len = mavlink_msg_to_send_buffer(buffer, &msg);
		int bytes_sent = sendto(target_sock, buffer, len, 0,
					(struct sockaddr*)&target_addr,
					sizeof(target_addr));

		if (bytes_sent <= 0)
			ERR("mavlog_thread(): Unable to send bytes");
	}
}

static void *heartbeat_thread(void *ptr)
{
	mavlink_message_t msg;

	while (m_run) {
		mavlink_msg_heartbeat_pack(MAVLOG_SYSTEM_ID,
					   MAVLOG_COMPONENT_ID, &msg,
					   MAV_TYPE_GENERIC,
					   MAV_AUTOPILOT_INVALID, 0, 0,
					   MAV_STATE_ACTIVE);
		m_msq_queue.add(msg);

		usleep(MAVLOG_HEARTBEAT_PERIOD_MS * 1000UL);
	}
}

void mavlog_init(void)
{
	DBG("mavlog_init()");

	m_initialized = true;
}

void mavlog_start(void)
{
	if (!m_initialized)
		return;

	DBG("mavlog_start()");

	m_run = true;

	int rc = pthread_create(&m_mavlink_thread, NULL, mavlink_thread, NULL);
	if (rc) {
		ERR("Unable to create thread: " << rc);
		return;
	}

	rc = pthread_create(&m_heartbeat_thread, NULL, heartbeat_thread, NULL);
	if (rc) {
		ERR("Unable to create thread: " << rc);
		return;
	}
}

void mavlog_send_motion(unsigned long timestamp, motion_t *p_motion)
{
	static uint64_t prev_t;

	if (!m_initialized || !m_run)
		return;

	if (prev_t == 0) {
		prev_t = (uint64_t)microseconds();
		return;
	}

	suseconds_t t1, t2;

	t1 = microseconds();

	uint64_t t = (uint64_t)microseconds();

	sensors_data_t sensors;
	memset(&sensors, 0, sizeof(sensors));
	sensors_read(&sensors);

	double dx_px, dy_px, dr_rad;
	uint8_t flow_quality;

	if (p_motion->affine_xform.cols == 3 && p_motion->affine_xform.rows == 2) {
		/* A = [sR|t] = [ s*cos(r), -s*sin(r), tx ]
		                  s*sin(r),  s*cos(r), ty ] */
		dx_px = p_motion->affine_xform.at<double>(0, 2);
		dy_px = p_motion->affine_xform.at<double>(1, 2);
		dr_rad = atan2(p_motion->affine_xform.at<double>(0, 1),
			       p_motion->affine_xform.at<double>(0, 0));

		/* TODO: vec_in too low -> low quality */
		flow_quality = (uint8_t)(255*p_motion->res.vec_in / p_motion->res.vec_good);
	} else {
		dx_px = 0;
		dy_px = 0;
		dr_rad = 0;
		flow_quality = 0;
	}

	/* TODO: Negative: Distance unknown */
	float ground_dist_m = (float)sensors.sonar.distance_mm / 1000.0f;

	/* TODO: Make this be the "Time in microseconds since the distance was sampled" */
	uint32_t ground_dist_dt = 0;

	/* Flow in dezi-pixels: */
	int16_t flow_x_10px = (int16_t)round(dx_px * 10.0);
	int16_t flow_y_10px = (int16_t)round(dy_px * 10.0);

	/* Flow in "radians"
	   (see https://groups.google.com/forum/#!topic/px4users/2JuTs5NXqq8 for
	   explanation): */
	float flow_x_rad = (float)(PX2M * dx_px);
	float flow_y_rad = (float)(PX2M * dy_px);

	/* Flow in meters per second: */
	float dt = (float)(t-prev_t) / 1000000.0f;
	float flow_x_m = (flow_x_rad / dt) * ground_dist_m;
	float flow_y_m = (flow_y_rad / dt) * ground_dist_m;;

	/* Gyro (note the Z axis represents rotation recovered from flow): */
	float gyro_x_rad = (float)((M_PI * sensors.gyro.x / 180.0) * dt);
	float gyro_y_rad = (float)((M_PI * sensors.gyro.y / 180.0) * dt);
	float gyro_z_rad = (float)dr_rad;
	int16_t gyro_t_cdeg = (int)(sensors.gyro.temperature * 100);

	/* OPTICAL_FLOW(): */
	mavlink_message_t msg;
	mavlink_msg_optical_flow_pack(MAVLOG_SYSTEM_ID, MAVLOG_COMPONENT_ID,
				      &msg, t, MAVLOG_SENSOR_ID, flow_x_10px,
				      flow_y_10px, flow_x_m, flow_y_m,
				      flow_quality, ground_dist_m);
	m_msq_queue.add(msg);

	/* OPTICAL_FLOW_RAD(): */
	mavlink_msg_optical_flow_rad_pack(MAVLOG_SYSTEM_ID, MAVLOG_COMPONENT_ID,
					  &msg, t, MAVLOG_SENSOR_ID,
					  (uint32_t)(t-prev_t), flow_x_rad,
					  flow_y_rad, gyro_x_rad, gyro_y_rad,
					  gyro_z_rad, gyro_t_cdeg, flow_quality,
					  ground_dist_dt, ground_dist_m);
	m_msq_queue.add(msg);

	prev_t = t;

	t2 = microseconds();

	DBG("mavlog_send_motion(): " << (t2-t1) << " us");
}

void mavlog_stop(void)
{
	if (!m_initialized)
		return;

	m_run = false;
	m_initialized = false;

	DBG("mavlog_stop()");
}
