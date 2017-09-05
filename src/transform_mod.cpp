/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "transform.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/video/video.hpp"

#define RANSAC_NITER		75
#define RANSAC_ERR_THRESH	1.5
#define RANSAC_INL_PROPORTION	0.5

using namespace cv;
using namespace std;

static void getRTMatrix(const Point2f* a, const Point2f* b, int count, Mat& M)
{
	CV_Assert( M.isContinuous() );

	/* Original OpenCV code: */
	double sa[4][4]={{0.}}, sb[4]={0.}, m[4];
	Mat A( 4, 4, CV_64F, sa ), B( 4, 1, CV_64F, sb );
	Mat MM( 4, 1, CV_64F, m );

	for( int i = 0; i < count; i++ ) {
		sa[0][0] += a[i].x*a[i].x + a[i].y*a[i].y;
		sa[0][2] += a[i].x;
		sa[0][3] += a[i].y;


		sa[2][1] += -a[i].y;
		sa[2][2] += 1;

		sa[3][0] += a[i].y;
		sa[3][1] += a[i].x;
		sa[3][3] += 1;

		sb[0] += a[i].x*b[i].x + a[i].y*b[i].y;
		sb[1] += a[i].x*b[i].y - a[i].y*b[i].x;
		sb[2] += b[i].x;
		sb[3] += b[i].y;
	}

	sa[1][1] = sa[0][0];
	sa[2][1] = sa[1][2] = -sa[0][3];
	sa[3][1] = sa[1][3] = sa[2][0] = sa[0][2];
	sa[2][2] = sa[3][3] = count;
	sa[3][0] = sa[0][3];

	solve( A, B, MM, DECOMP_EIG );

	double* om = M.ptr<double>();
	om[0] = om[4] = m[0];
	om[1] = -m[1];
	om[3] = m[1];
	om[2] = m[2];
	om[5] = m[3];
}

Mat ransac(vector<Point2f>& src, vector<Point2f>& dst, int *p_good_count, int seed)
{
	RNG rng(microseconds() * seed);
	int count = src.size();

	vector<int> inl_idx(count);

	int k;

	/* Sampled points: */
	Point2f sam_src[2];
	Point2f sam_dst[2];
	int sam_idx[2];

	/* Best solution: */
	vector<int> best_inl_idx(count);
	int best_inl_count = 0;

	Mat A(2, 3, CV_64F);

	double err_thresh = RANSAC_ERR_THRESH * RANSAC_ERR_THRESH; /* [px^2] */

	for (k = 0; k < RANSAC_NITER; k++) {
		/* Select two random samples: */
		sam_idx[0] = rng.uniform(0, count);
		do {
			sam_idx[1] = rng.uniform(0, count);
		} while (sam_idx[0] == sam_idx[1]);

		for (int i = 0; i < 2; i++) {
			sam_src[i] = src[sam_idx[i]];
			sam_dst[i] = dst[sam_idx[i]];
		}

		/* Estimate model using the two samples: */
		getRTMatrix(sam_src, sam_dst, 2, A);

		/* Evaluate the model and identify inliers: */
		const double* p_a = A.ptr<double>();
		int inl_count = 0;
		for(int i = 0; i < count; i++) {
			double dx = p_a[0]*src[i].x + p_a[1]*src[i].y +
				    p_a[2] - dst[i].x;
			double dy = p_a[3]*src[i].x + p_a[4]*src[i].y +
				    p_a[5] - dst[i].y;

			if ((dx*dx + dy*dy) < err_thresh)
				inl_idx[inl_count++] = i;
		}

		/*if (inl_count >= (RANSAC_INL_PROPORTION * count)) {
			best_inl_count = inl_count;
			best_inl_idx = inl_idx;
			break;
		}*/
		if (inl_count > best_inl_count) {
			best_inl_count = inl_count;
			best_inl_idx = inl_idx;
		}
	}

	/* Re-estimate the model using the largest set of inliers: */
	vector<Point2f> inl_src(best_inl_count);
	vector<Point2f> inl_dst(best_inl_count);

	for (int i = 0; i < best_inl_count; i++) {
		inl_src[i] = src[best_inl_idx[i]];
		inl_dst[i] = dst[best_inl_idx[i]];
	}

	getRTMatrix(&inl_src[0], &inl_dst[0], best_inl_count, A);
	*p_good_count = best_inl_count;

	return A;
}

class Parallel_process : public cv::ParallelLoopBody
{
private:
	vector<Point2f>& m_src1;
	vector<Point2f>& m_src2;
	int m_nthreads;

	Mat *m_ms;
	int *m_gs;

public:
	Parallel_process(vector<Point2f>& src1, vector<Point2f>& src2, int nthreads)
	: m_src1(src1), m_src2(src2) {
		m_nthreads = nthreads;
		m_ms = new Mat[m_nthreads];
		m_gs = new int[m_nthreads];
	}

	~Parallel_process()
	{
		delete [] m_ms;
		delete [] m_gs;
	}

	virtual void operator()(const cv::Range& range) const
	{
		for(int i = range.start; i < range.end; i++)
			m_ms[i] = ransac(m_src1, m_src2, &m_gs[i], i);
	}

	void result(Mat& mat, int *p_good_count)
	{
		int best_id = -1;
		int best_count = 0;
		for (int i = 0; i < m_nthreads; i++) {
			if (m_gs[i] > best_count) {
				best_count = m_gs[i];
				best_id = i;
			}
		}

		if (best_id >= 0) {
			m_ms[best_id].copyTo(mat);
			*p_good_count = best_count;
			DBG("Found solution with " << best_count << " inliers");
		} else {
			*p_good_count = 0;
			DBG("Found no solution.");
		}
	}
};


Mat transform_estimate_rigid(vector<Point2f>& src, vector<Point2f>& dst, int *p_good_count)
{
	int nthreads = 3;
	*p_good_count = 0;

	setNumThreads(nthreads);
	Parallel_process p(src, dst, nthreads);
	parallel_for_(cv::Range(0, nthreads), p, nthreads);

	Mat tform;

	p.result(tform, p_good_count);

	if (*p_good_count == 0)
		return Mat();
	else
		return tform;
}
