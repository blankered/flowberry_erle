#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "common.h"
#include "opencv2/core/utility.hpp"

cv::Mat transform_estimate_rigid(std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dst, int *p_good_count);

#endif
