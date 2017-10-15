#include "draw.h"

#include <opencv2/core/utility.hpp>

using namespace cv;
using namespace std;

static bool m_enabled;

static Mat m_img;
static Mat m_colormap_img;

void draw_init(int width, int height)
{
	m_img.create(height, width, CV_8UC3);

	/* Create colormap: */
	Mat colormap_in(64, 256, CV_8UC1);

	for (int i = 0; i < colormap_in.rows; i++)
		for (int j = 0; j < colormap_in.cols; j++)
			colormap_in.at<uchar>(i, j) = (uint8_t)j;

	applyColorMap(colormap_in, m_colormap_img, COLORMAP_JET);

	m_enabled = true;
}

void draw_prepare(cv_img& img)
{
	if (!m_enabled)
		return;

	cvtColor(img.mat(), m_img, CV_GRAY2RGB);
}

void draw_imv(cv_imv& imv, int sad_limit)
{
	if (!m_enabled)
		return;

	suseconds_t t1, t2;
	int i, j;

	t1 = microseconds();

	cv_imv_t *p_imv = imv.imv();
	int mbx = imv.mbx();
	int mby = imv.mby();

	for (j = 0; j < mby; j++) {
		for (i = 0; i < mbx; i++) {
			cv_imv_t *p_vec = p_imv + (i+(mbx+1)*j);

			if (p_vec->x == 0 && p_vec->y == 0)
				continue;

			if (p_vec->sad > sad_limit)
				continue;

			int x = i*16 + 8;
			int y = j*16 + 8;

			float intensity = p_vec->sad;
			intensity = round(255 * intensity / sad_limit);

			if (intensity > 255)
				intensity = 255;

			uint8_t *ptr = m_colormap_img.ptr<uchar>(0);
			uint8_t idx = 3*(uint8_t)intensity;

			arrowedLine(m_img, Point(x+p_vec->x, y+p_vec->y),
				    Point(x, y),
				    Scalar(ptr[idx], ptr[idx+1], ptr[idx+2]));
		}
	}

	t2 = microseconds();

	DBG("[draw] draw_imv(): " << (t2-t1) << " us");
}

Mat& draw_get_image(void)
{
	return m_img;
}

Mat& draw_get_colormap(void)
{
	return m_colormap_img;
}
