#ifndef MAVLOG_H
#define MAVLOG_H

#include "common.h"
#include "motion.h"

void mavlog_init(void);
void mavlog_start(void);

void mavlog_send_motion(unsigned long timestamp, motion_t *p_motion);

void mavlog_stop(void);

#endif