#ifndef SERIAL_H
#define SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

bool sonar_init(char *p_path);
bool sonar_read(int *p_distance_mm);
bool sonar_close(void);

#ifdef __cplusplus
}
#endif

#endif
