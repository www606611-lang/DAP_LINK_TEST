#include "wheel_odometry.h"

#include "encoder_input.h"

#include <stddef.h>

static wheel_odometry_snapshot_t g_snapshot;
static uint32_t g_last_update_ms;
static bool g_has_sample;

static int32_t wheel_odometry_average(int32_t left, int32_t right);

void WheelOdometry_Init(uint32_t now_ms)
{
    g_snapshot.left_count = 0;
    g_snapshot.right_count = 0;
    g_snapshot.average_count = 0;
    g_snapshot.left_delta_count = 0;
    g_snapshot.right_delta_count = 0;
    g_snapshot.average_delta_count = 0;
    g_snapshot.sync_error_count = 0;
    g_snapshot.update_count = 0U;
    g_snapshot.last_interval_ms = 0U;
    g_snapshot.max_interval_ms = 0U;
    g_snapshot.ready = false;
    g_last_update_ms = now_ms;
    g_has_sample = false;
}

void WheelOdometry_Task(uint32_t now_ms)
{
    encoder_input_snapshot_t left;
    encoder_input_snapshot_t right;
    uint32_t elapsed_ms;
    int32_t previous_average;
    int32_t previous_left;
    int32_t previous_right;

    elapsed_ms = now_ms - g_last_update_ms;
    if (elapsed_ms < WHEEL_ODOMETRY_UPDATE_INTERVAL_MS) {
        return;
    }
    if (!EncoderInput_GetSnapshot(ENCODER_INPUT_0, &left) ||
        !EncoderInput_GetSnapshot(ENCODER_INPUT_1, &right)) {
        g_snapshot.ready = false;
        return;
    }

    previous_average = g_snapshot.average_count;
    previous_left = g_snapshot.left_count;
    previous_right = g_snapshot.right_count;
    g_last_update_ms = now_ms;
    g_snapshot.last_interval_ms = elapsed_ms;
    if (elapsed_ms > g_snapshot.max_interval_ms) {
        g_snapshot.max_interval_ms = elapsed_ms;
    }
    g_snapshot.left_count = left.count;
    g_snapshot.right_count = right.count;
    g_snapshot.average_count = wheel_odometry_average(
        left.count, right.count);
    g_snapshot.sync_error_count = left.count - right.count;
    if (g_has_sample) {
        g_snapshot.left_delta_count = left.count - previous_left;
        g_snapshot.right_delta_count = right.count - previous_right;
        g_snapshot.average_delta_count =
            g_snapshot.average_count - previous_average;
    } else {
        g_snapshot.left_delta_count = 0;
        g_snapshot.right_delta_count = 0;
        g_snapshot.average_delta_count = 0;
        g_has_sample = true;
    }
    g_snapshot.update_count++;
    g_snapshot.ready = true;
}

bool WheelOdometry_GetSnapshot(wheel_odometry_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    *snapshot = g_snapshot;
    return true;
}

static int32_t wheel_odometry_average(int32_t left, int32_t right)
{
    int64_t sum = (int64_t) left + (int64_t) right;

    return (int32_t) (sum / 2);
}
