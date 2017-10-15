#include "gui.h"

#include <stdio.h>
#include <string.h>
#include "util.h"

#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

static bool m_enabled;

const char *m_src_win_title = "Source";
const char *m_colormap_win_title = "SAD Colormap";
const char *m_sad_trackbar_title = "SAD Threshold";

void gui_init(bool enable, Mat& colormap_img, int *p_sad_limit, int sad_limit_max)
{
	m_enabled = enable;

	if (!m_enabled)
		return;

	/* Initialize UI: */
	namedWindow(m_colormap_win_title, WINDOW_AUTOSIZE);
	imshow(m_colormap_win_title, colormap_img);

	namedWindow(m_src_win_title, WINDOW_AUTOSIZE);
	createTrackbar(m_sad_trackbar_title, m_src_win_title, p_sad_limit, sad_limit_max);

	waitKey(1000);
}

void gui_display(Mat& image)
{
	if (!m_enabled)
		return;

	imshow(m_src_win_title, image);
}
