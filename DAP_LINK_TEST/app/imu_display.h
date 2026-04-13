#ifndef APP_IMU_DISPLAY_H
#define APP_IMU_DISPLAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void imu_display_init(uint32_t now_ms);
void imu_display_task(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
