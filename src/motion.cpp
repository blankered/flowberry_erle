#include "motion.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/video.hpp>
#include "sensors.h"
#include "transform.h"
#include "undistort.h"

using namespace cv;
using namespace std;

void motion_init(void)
{
	undistort_init();
}

void motion_calc_from_imv(cv_imv& imv, motion_t *p_motion, int sad_limit)
{
	static Mat1f empty_homography;
	static suseconds_t t_max;
	suseconds_t t1, t2, t3, t;

	t1 = microseconds();

	vector<Point2f> pts_src;
	vector<Point2f> pts_dst;

	int count = 0;

	cv_imv_t *p_imv = imv.imv();
	int mbx = imv.mbx();
	int mby = imv.mby();

	for (int j = 0; j < mby; j++) {
		for (int i = 0; i < mbx; i++) {
			cv_imv_t *p_vec = p_imv + (i+(mbx+1)*j);

			if (p_vec->x == 0 && p_vec->y == 0)
				continue;

			//if (p_vec->sad > sad_limit)
			//	continue;

			int x = i*16 + 8;
			int y = j*16 + 8;

			pts_src.push_back(Point2f(x+p_vec->x, y+p_vec->y));
			pts_dst.push_back(Point2f(x, y));
			count++;
		}
	}

	t2 = microseconds();


	DBG("motion_calc_from_imv(): " << count << " vectors");

	p_motion->dx = 0;
	p_motion->dy = 0;

	p_motion->res.vec_in = count;
	p_motion->res.vec_good = 0;

	if (count > 0) {
		undistort_process_flow(pts_src, pts_dst);
		sensors_compensate(pts_src, pts_dst);
	}

	if (count >= 3)
		p_motion->affine_xform = transform_estimate_rigid(pts_src, pts_dst, &p_motion->res.vec_good);
	else
		p_motion->affine_xform = empty_homography;

	t3 = microseconds();

	t = t3 - t1;
	if (t > t_max)
		t_max = t;

	DBG("motion_calc_from_imv(): Estimated [A|b] is:" << endl << p_motion->affine_xform);

	DBG("motion_calc_from_imv(): " << t << " (copy: " << (t2-t1) << ", max: " << t_max << ") us");	
}
