#ifndef APP_LINE_SENSOR_BRINGUP_H
#define APP_LINE_SENSOR_BRINGUP_H

#include "line_sensor.h"

#include <stdbool.h>
#include <stdint.h>

void LineSensorBringup_Init(uint32_t now_ms);
void LineSensorBringup_Task(uint32_t now_ms);
bool LineSensorBringup_RequestCalibration(void);
bool LineSensorBringup_GetSnapshot(line_sensor_snapshot_t *snapshot);
const char *LineSensorBringup_GetStateText(void);

#endif
