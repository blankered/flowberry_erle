#ifndef CV_IMG_H
#define CV_IMG_H

#include <stdint.h>
#include "opencv2/imgproc/imgproc.hpp"

class cv_img
{
	cv::Mat m_mat;
	int m_width;
	int m_height;
	unsigned long m_timestamp;

public:
	cv_img(uint8_t *p_buffer, int width, int height, unsigned long timestamp)
	{
		m_width = width;
		m_height = height;
		m_timestamp = timestamp;

		int len = m_width * m_height * sizeof(uint8_t);
		m_mat.create(m_height, m_width, CV_8UC1);
		memcpy(m_mat.ptr<uchar>(0), p_buffer, len);		
	}

	~cv_img()
	{
		m_mat.release();
	}

	cv::Mat& mat()
	{
		return m_mat;
	}

	unsigned long timestamp()
	{
		return m_timestamp;
	}

	int width()
	{
		return m_width;
	}

	int height()
	{
		return m_height;
	}
};

#endif
