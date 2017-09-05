#include "util.h"

suseconds_t microseconds()
{
	static struct timeval tv;
	gettimeofday(&tv, NULL);

	return ((tv.tv_sec * 1000000UL) + tv.tv_usec);
}
