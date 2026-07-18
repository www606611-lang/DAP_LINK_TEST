#ifndef CONTROL_WHEEL_ODOMETRY_H
#define CONTROL_WHEEL_ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>

#define WHEEL_ODOMETRY_UPDATE_INTERVAL_MS 10U

typedef struct {
    int32_t left_count;
    int32_t right_count;
    int32_t average_count;
    int32_t left_delta_count;
    int32_t right_delta_count;
    int32_t average_delta_count;
    int32_t sync_error_count;
    uint32_t update_count;
    uint32_t last_interval_ms;
    uint32_t max_interval_ms;
    bool ready;
} wheel_odometry_snapshot_t;

void WheelOdometry_Init(uint32_t now_ms);
void WheelOdometry_Task(uint32_t now_ms);
bool WheelOdometry_GetSnapshot(wheel_odometry_snapshot_t *snapshot);

#endif
