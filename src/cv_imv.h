#ifndef CV_IMV_H
#define CV_IMV_H

#include "common.h"

/* TODO: Packed structure? */
typedef struct {
	int8_t x;
	int8_t y;
	uint16_t sad;
} cv_imv_t;

typedef struct {
	int avg_sad;
	int good_count;
	double avg_x;
	double avg_y;
} cv_imv_stats_t;

class cv_imv
{
	cv_imv_t *m_imv = NULL;
	size_t m_size;
	int m_mbx;
	int m_mby;
	unsigned long m_timestamp;

public:
	cv_imv(uint8_t *p_buffer, int mbx, int mby, unsigned long timestamp)
	{
		m_mbx = mbx;
		m_mby = mby;
		m_size = (m_mbx+1) * (m_mby);
		m_timestamp = timestamp;
		m_imv = new cv_imv_t[m_size];
		memcpy(m_imv, p_buffer, m_size * sizeof(cv_imv_t));
	}

	cv_imv(cv_imv const& copy)
	{
		/* Comes from http://stackoverflow.com/a/255744 */
		m_size = copy.m_size;
		m_mbx = copy.m_mbx;
		m_mby = copy.m_mby;
		m_imv = new cv_imv_t[m_size];
		std::copy(&copy.m_imv[0], &copy.m_imv[copy.m_size], m_imv);
	}

	cv_imv& operator=(cv_imv rhs)
	{
		rhs.swap(*this);
		return *this;
	}

	void swap(cv_imv& s) noexcept
	{
		std::swap(m_imv, s.m_imv);
		std::swap(m_size, s.m_size);
		std::swap(m_mbx, s.m_mbx);
		std::swap(m_mby, s.m_mby);
		std::swap(m_timestamp, s.m_timestamp);
	}

	~cv_imv()
	{
		delete [] m_imv;
	}

	cv_imv_stats_t stats()
	{
		suseconds_t t1, t2;
		cv_imv_stats_t stats;

		t1 = microseconds();

		memset(&stats, 0, sizeof(stats));

		int count = 0;

		for (int j = 0; j < m_mby; j++) {
			for (int i = 0; i < m_mbx; i++) {
				cv_imv_t *p_vec = m_imv + (i+(m_mbx+1)*j);

				if (p_vec->x == 0 && p_vec->y == 0 && p_vec->sad == 0)
					continue;

				stats.avg_sad += p_vec->sad;
				count++;
			}
		}

		if (count > 0) {
			stats.avg_sad /= count;
			stats.avg_sad *= 1.1;
		} else {
			stats.avg_sad = 0;
		}

		count = 0;

		for (int j = 0; j < m_mby; j++) {
			for (int i = 0; i < m_mbx; i++) {
				cv_imv_t *p_vec = m_imv + (i+(m_mbx+1)*j);

				if (p_vec->x == 0 && p_vec->y == 0)
					continue;

				if (p_vec->sad > stats.avg_sad)
					continue;

				stats.avg_x += p_vec->x;
				stats.avg_y += p_vec->y;
				count++;
			}
		}

		if (count > 0) {
			stats.avg_x /= count;
			stats.avg_y /= count;
		} else {
			stats.avg_x = 0;
			stats.avg_y = 0;
		}

		stats.good_count = count;

		t2 = microseconds();

		DBG("stats(): avg_sad: " << stats.avg_sad);
		DBG("stats(): avg_x: " << stats.avg_x);
		DBG("stats(): avg_y: " << stats.avg_y);
		DBG("stats(): " << (t2-t1) << " us");

		return stats;
	}

	cv_imv_t *imv()
	{
		return m_imv;
	}

	unsigned long timestamp()
	{
		return m_timestamp;
	}

	int mbx()
	{
		return m_mbx;
	}

	int mby()
	{
		return m_mby;
	}
};

#endif
