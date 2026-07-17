#include "line_sensor_bringup.h"

static bool g_calibration_requested;

void LineSensorBringup_Init(uint32_t now_ms)
{
    g_calibration_requested = false;
    LineSensor_Init(now_ms);
}

void LineSensorBringup_Task(uint32_t now_ms)
{
    if (g_calibration_requested) {
        g_calibration_requested = false;
        (void) LineSensor_RequestCalibration(now_ms);
    }
    LineSensor_Task(now_ms);
}

bool LineSensorBringup_RequestCalibration(void)
{
    line_sensor_snapshot_t snapshot;

    if (g_calibration_requested ||
        !LineSensor_GetSnapshot(&snapshot) ||
        (snapshot.state == LINE_SENSOR_STATE_BOOT_WAIT) ||
        (snapshot.state == LINE_SENSOR_STATE_CALIBRATION_ON) ||
        (snapshot.state == LINE_SENSOR_STATE_CALIBRATION_OFF_WAIT)) {
        return false;
    }
    g_calibration_requested = true;
    return true;
}

bool LineSensorBringup_GetSnapshot(line_sensor_snapshot_t *snapshot)
{
    return LineSensor_GetSnapshot(snapshot);
}

const char *LineSensorBringup_GetStateText(void)
{
    return LineSensor_GetStateText();
}
