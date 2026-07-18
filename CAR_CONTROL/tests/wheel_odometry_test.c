#include "wheel_odometry.h"

#include "encoder_input.h"

#include <assert.h>

static encoder_input_snapshot_t g_encoder[ENCODER_INPUT_COUNT];

static wheel_odometry_snapshot_t snapshot(void)
{
    wheel_odometry_snapshot_t value;

    assert(WheelOdometry_GetSnapshot(&value));
    return value;
}

int main(void)
{
    wheel_odometry_snapshot_t value;

    g_encoder[ENCODER_INPUT_0].count = 100;
    g_encoder[ENCODER_INPUT_1].count = 100;
    WheelOdometry_Init(0U);
    WheelOdometry_Task(10U);
    value = snapshot();
    assert(value.ready);
    assert(value.average_count == 100);
    assert(value.average_delta_count == 0);

    g_encoder[ENCODER_INPUT_0].count = 150;
    g_encoder[ENCODER_INPUT_1].count = 130;
    WheelOdometry_Task(20U);
    value = snapshot();
    assert(value.average_count == 140);
    assert(value.left_delta_count == 50);
    assert(value.right_delta_count == 30);
    assert(value.average_delta_count == 40);
    assert(value.sync_error_count == 20);

    WheelOdometry_Task(25U);
    value = snapshot();
    assert(value.update_count == 2U);
    return 0;
}

bool EncoderInput_GetSnapshot(
    encoder_input_id_t id, encoder_input_snapshot_t *snapshot_value)
{
    if ((id >= ENCODER_INPUT_COUNT) || (snapshot_value == NULL)) {
        return false;
    }
    *snapshot_value = g_encoder[id];
    return true;
}
