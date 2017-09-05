#ifndef MOTION_H
#define MOTION_H

#include "opencv2/core/utility.hpp"

#include "common.h"
#include "flow.h"
#include "ffmpeg.h"
#include "cv_imv.h"

typedef struct {
	cv::Mat affine_xform;
	double dx;
	double dy;

	struct {
		int vec_in;
		int vec_good;
	} res;
} motion_t;

void motion_init(void);
void motion_calc_from_imv(cv_imv& imv, motion_t *p_motion, int sad_limit);

#endif
