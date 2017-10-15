#ifndef DRAW_H
#define DRAW_H

#include "common.h"
#include "cv_img.h"
#include "cv_imv.h"

#include "opencv2/imgproc/imgproc.hpp"

void draw_init(int width, int height);

void draw_prepare(cv_img& img);
void draw_imv(cv_imv& imv, int sad_limit);

cv::Mat& draw_get_image(void);
cv::Mat& draw_get_colormap(void);

#endif
