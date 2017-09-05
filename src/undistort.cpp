#include "undistort.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#define CALIBRATION_FILENAME	"flowberry_camera_calib.xml"

#define UNDISTORT_USE_MAPS /* Only valid for integer flow (IMV from the CME) */

using namespace std;
using namespace cv;

static bool m_initialized;

static int m_imw;
static int m_imh;

static Mat m_camera_mat;
static Mat m_camera_new_mat;
static Mat m_dist_coeffs;

#ifdef UNDISTORT_USE_MAPS
static Mat m_map_x;
static Mat m_map_y;
#endif

void undistort_init(void)
{
	suseconds_t t1, t2;

	t1 = microseconds();

	FileStorage fs(CALIBRATION_FILENAME, FileStorage::READ);

	if (!fs.isOpened()) {
		ERR("undistort_init(): Can't open " << CALIBRATION_FILENAME);
		return;
	}


	fs["camera_matrix"] >> m_camera_mat;
	fs["distortion_coefficients"] >> m_dist_coeffs;

	m_imw = (int)fs["image_width"];
	m_imh = (int)fs["image_height"];

	fs.release();

	DBG("Calibration for " << m_imw << "x" << m_imh << " image:" << endl
	    << "K = " << m_camera_mat << endl
	    << "D = " << m_dist_coeffs);

	Size sz(m_imw, m_imh);
	m_camera_new_mat = getOptimalNewCameraMatrix(m_camera_mat,m_dist_coeffs,
						     sz, 1, sz, 0);

#ifdef UNDISTORT_USE_MAPS
	initUndistortRectifyMap(m_camera_mat, m_dist_coeffs, Mat(),
				m_camera_new_mat, sz, CV_32FC1, m_map_x,
				m_map_y);
#endif

	t2 = microseconds();

	/* TODO: Also check camera matrix and dist. coefficients: */
	if (m_imw > 0 && m_imh > 0)
		m_initialized = true;

	DBG("undistort_init(): " << (t2-t1) << "us");
}

static void remap_point(Point2f& p)
{
#ifdef UNDISTORT_USE_MAPS
	int x = (float)p.x;
	int y = (float)p.y;

	p.x = m_map_x.at<float>(y,x);
	p.y = m_map_y.at<float>(y,x);
#endif
}

void undistort_process_flow(vector<Point2f>& src, vector<Point2f>& dst)
{
	if (!m_initialized)
		return;

	suseconds_t t1, t2;
	int count;

	t1 = microseconds();

#ifdef UNDISTORT_USE_MAPS
	count = 0;

	/* TODO: Use an iterator */
	int i;
	for (i = 0; i < src.size(); i++) {
		/* TODO: Decide what to do with points otside the range */
		if (src[i].x >= 0 && src[i].x <= m_imw &&
		    src[i].y >= 0 && src[i].y <= m_imh &&
		    dst[i].x >= 0 && dst[i].x <= m_imw &&
		    dst[i].y >= 0 && dst[i].y <= m_imh) {
			remap_point(src[i]);
			remap_point(dst[i]);
			count++;
		}
	}
#else
	undistortPoints(src, src, m_camera_mat, m_dist_coeffs, Mat(),
			m_camera_new_mat);
	undistortPoints(dst, dst, m_camera_mat, m_dist_coeffs, Mat(),
			m_camera_new_mat);
	count = src.size();
#endif

	t2 = microseconds();

	DBG("undistort_process_flow(): Undistorted " << count << " of "
	    << src.size() << " vectors (" << (t2-t1) << " us)");
}
