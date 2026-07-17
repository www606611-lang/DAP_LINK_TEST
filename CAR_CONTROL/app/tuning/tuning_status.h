#ifndef APP_TUNING_TUNING_STATUS_H
#define APP_TUNING_TUNING_STATUS_H

#include <stdint.h>

void speed_tuning_send_config(void);
void speed_tuning_send_status(void);
void speed_tuning_send_position_config(void);
void speed_tuning_send_position_status(void);
void speed_tuning_send_imu_status(uint32_t now_ms);
void speed_tuning_send_yaw_config(void);
void speed_tuning_send_yaw_status(void);
void speed_tuning_send_heading_config(void);
void speed_tuning_send_heading_status(void);
void speed_tuning_send_line_config(void);
void speed_tuning_send_line_status(uint32_t now_ms);

#endif
