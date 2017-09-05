#ifndef GUI_H
#define GUI_H

#include "common.h"

#include "opencv2/imgproc/imgproc.hpp"

void gui_init(bool enable, cv::Mat& colormap_img, int *p_sad_limit, int sad_limit_max);
void gui_display(cv::Mat& image);

#endif
