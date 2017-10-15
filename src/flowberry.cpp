#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <unistd.h>
#include <iostream>
#include <pthread.h>

#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/optflow.hpp>

#include "cv.h"
#include "cv_queue.h"
#include "cv_img.h"
#include "cv_imv.h"
#include "gui.h"
#include "draw.h"
#include "motion.h"
#include "sensors.h"
#include "mavlog.h"
#include "transform.h"
#include "raspividcv.h"

#define CONFIG_ENABLE_SONAR	true

using namespace cv;
using namespace std;
using namespace cv::optflow;

static struct {
	int width;
	int height;
	int mbx;
	int mby;
} m_img;

static volatile bool m_initialized;
static pthread_t m_thread;

static cv_queue<cv_img*> m_img_queue;
static cv_queue<cv_imv*> m_imv_queue;

static bool m_use_gui;

static suseconds_t m_frame_delay;

static void algo_imv(int sad_limit)
{
	int cnt = 0;
	unsigned long frame = 0;
	unsigned long skipped_frames = 0;
	suseconds_t t1, t2, t, prev_t;
	motion_t motion;
	sensors_data_t sensors;

	motion_init();
	mavlog_init();
	mavlog_start();

	while (1) {
		cv_img *img;

		if (m_use_gui)
			img = m_img_queue.remove();
		cv_imv *imv = m_imv_queue.remove();

		t1 = microseconds();

		cv_imv_stats_t stats = imv->stats();
		sad_limit = stats.avg_sad;

		DBG("sad_limit = " << sad_limit);

		motion_calc_from_imv(*imv, &motion, sad_limit);
		sensors_read(&sensors);

		if (m_use_gui && cnt++ == 10) {
			draw_prepare(*img);
			draw_imv(*imv, sad_limit);
			gui_display(draw_get_image());
			cnt = 0;
		}

		motion.dx = stats.avg_x;
		motion.dy = stats.avg_y;

		mavlog_send_motion((microseconds()-t1), &motion);

		if (m_use_gui)
			waitKey(1);

		if (m_use_gui)
			delete img;
		delete imv;

		t2 = microseconds();
		t = t2-t1;

		DBG("[algo_imv] " << frame << " Finished, duration: " << t << " us (skipped " << skipped_frames << " frames)");

#ifndef DEBUG
		printf("\rFrame %d (%lu us, %d lost)  ", frame, t, skipped_frames);
		fflush(stdout);
#endif

		if (t >= m_frame_delay) {
			int frames_to_skip = t/m_frame_delay;
			while (frames_to_skip > 0) {
				if (m_use_gui)
					m_img_queue.remove();
				m_imv_queue.remove();
				skipped_frames++;
				frames_to_skip--;
			}
		}

		prev_t = t;
		frame++;
	}
}

static void *process_thread(void *ptr)
{
	int cnt = 0;
	int sad_limit = 500;

	draw_init(m_img.width, m_img.height);
	gui_init(m_use_gui, draw_get_colormap(), &sad_limit, 2000);

	algo_imv(sad_limit);
}

void cv_init(int width, int height, int fps, int fmt)
{
	DBG("cv_init(" << width << ", " << height << ", " << fps << ")");

	int imv_size = sizeof(cv_imv_t);
	if (imv_size != 4) {
		ERR("Error: sizeof(cv_imv_t) = " << imv_size << " instead of 4");
		return;
	}

	if (fmt != 3) {
		ERR("Format " << fmt << " is not grayscale");
		return;
	}

	m_img.width = width;
	m_img.height = height;

	if (m_img.width % 16 != 0) {
		DBG("Width " << m_img.width << " is not a multiple of 16");
		m_img.width += (16-(m_img.width % 16));
		DBG("Width increased to " << m_img.width);
	}

	if (m_img.height % 16 != 0) {
		DBG("Height " << m_img.height << " is not a multiple of 16");
		m_img.height += (16-(m_img.height % 16));
		DBG("Height increased to " << m_img.height);
	}

	m_img.mbx = m_img.width/16;
	m_img.mby = m_img.height/16;

	sensors_init(CONFIG_ENABLE_SONAR);

	/* Start thread: */
	int rc = pthread_create(&m_thread, NULL, process_thread, NULL);
	if (rc) {
		ERR("Unable to create thread: " << rc);
		return;
	}

	sensors_start();

	m_initialized = true;
}

void cv_process_img(uint8_t *p_buffer, int length, int64_t timestamp)
{
	if (!m_use_gui)
		return;

	static int64_t prev_timestamp;
	suseconds_t t1, t2;

	if (!m_initialized)
		return;

	if (length != m_img.width*m_img.height) {
		ERR("Wrong img length: " << length);
		m_initialized = false;
		return;
	}

	t1 = microseconds();
	cv_img *img = new cv_img(p_buffer, m_img.width, m_img.height, t1);
	m_img_queue.add(img);
	t2 = microseconds();

	DBG("cv_process_img(p_buffer, " << length << ") [dts " <<
	    (timestamp-prev_timestamp) << "]: " << (t2-t1) << " us");

	prev_timestamp = timestamp;
}

void cv_process_imv(uint8_t *p_buffer, int length, int64_t timestamp)
{
	static int64_t prev_timestamp;
	suseconds_t t1, t2;

	if (!m_initialized)
		return;

	if (length != (m_img.mbx+1)*(m_img.mby)*sizeof(cv_imv_t)) {
		ERR("Wrong imv length: " << length);
		m_initialized = false;
		return;
	}

	t1 = microseconds();
	cv_imv *imv = new cv_imv(p_buffer, m_img.mbx, m_img.mby, t1);
	m_imv_queue.add(imv);
	t2 = microseconds();

	DBG("cv_process_imv(p_buffer, " << length << ") [dts " <<
	    (timestamp-prev_timestamp) << "]: " << (t2-t1) << " us");

	prev_timestamp = timestamp;
}

void cv_close(void)
{
	DBG("cv_close()");
	m_initialized = false;
	sensors_stop();
	mavlog_stop();
}

int main(int argc, const char **argv)
{
	/* Prepare arguments for RaspiVid: */
	/* 640x480, 1920x1080, 1280x720 */
	const char *rargv[] = { argv[0], "-v", "-md", "4", "-w", "480", "-h",
				"480", "-fps", "30", "-t", "0", "-o",
				"/dev/null", "-x", "/dev/null", "-r",
				"/dev/null", "-rf", "gray", "-g", "0" };

	int rargc = sizeof(rargv) / sizeof(rargv[0]);

	if (argc >= 2) {
		rargv[9] = argv[1];
		int fps = atoi(argv[1]);
		m_frame_delay = 1000000UL/fps;
		printf("FPS: %d, delay: %lu us\n", fps, m_frame_delay);

		if (argc == 3) {
			if (strcmp("gui", argv[2]) == 0)
				m_use_gui = true;
		}
	} else {
		fprintf(stderr, "Usage: %s <fps>, [gui]\n", argv[0]);
		return 1;
	}

	int ret = raspividcv_main(rargc, rargv);

	volatile unsigned int i = 0;
	while (1) {
		i++;
		usleep(1000000);
	}

	return ret;
}
