#ifndef UNDISTORT_H
#define UNDISTORT_H

#include "common.h"
#include <opencv2/core/utility.hpp>

void undistort_init(void);
void undistort_process_flow(std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dst);

#endif
