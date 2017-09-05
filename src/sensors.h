#ifndef SENSORS_H
#define SENSORS_H

#include "common.h"

#include "opencv2/core/utility.hpp"

typedef struct sensors_data {
	struct {
		double x;
		double y;
		double z;
		int temperature;
	} gyro;
	struct {
		int distance_mm;
	} sonar;
} sensors_data_t;

bool sensors_init(bool enable_sonar);
bool sensors_start(void);
void sensors_read(sensors_data_t *p_data);
void sensors_stop(void);

void sensors_compensate(std::vector<cv::Point2f>& pts_src, std::vector<cv::Point2f>& pts_dst);

#endif
