#ifndef APP_LINE_TRACKING_BRINGUP_TEST_H
#define APP_LINE_TRACKING_BRINGUP_TEST_H

#include "wheel_line_tracking_control.h"

#include <stdbool.h>
#include <stdint.h>

#define LINE_TRACKING_BRINGUP_OUTPUT_MIN       100U
#define LINE_TRACKING_BRINGUP_OUTPUT_MAX      1000U
#define LINE_TRACKING_BRINGUP_DURATION_MIN_MS  500U
#define LINE_TRACKING_BRINGUP_DURATION_MAX_MS 10000U
#define LINE_TRACKING_BRINGUP_BASE_SPEED_MIN_PPS 100.0f

typedef enum {
    LINE_TRACKING_BRINGUP_TEST_LOCKED = 0,
    LINE_TRACKING_BRINGUP_TEST_READY,
    LINE_TRACKING_BRINGUP_TEST_RUNNING,
    LINE_TRACKING_BRINGUP_TEST_COMPLETE,
    LINE_TRACKING_BRINGUP_TEST_ABORTED
} line_tracking_bringup_test_state_t;

typedef enum {
    LINE_TRACKING_BRINGUP_CONFIG_OK = 0,
    LINE_TRACKING_BRINGUP_CONFIG_BAD_ARGUMENT,
    LINE_TRACKING_BRINGUP_CONFIG_BAD_RANGE,
    LINE_TRACKING_BRINGUP_CONFIG_BUSY
} line_tracking_bringup_config_result_t;

typedef struct {
    wheel_line_tracking_config_t control;
    float base_speed_pps;
    uint16_t output_limit_permille;
    uint32_t duration_ms;
} line_tracking_bringup_config_t;

void LineTrackingBringupTest_Init(bool reset_locked);
void LineTrackingBringupTest_Task(uint32_t now_ms);
line_tracking_bringup_config_result_t LineTrackingBringupTest_SetConfig(
    const line_tracking_bringup_config_t *config);
bool LineTrackingBringupTest_GetConfig(
    line_tracking_bringup_config_t *config);
bool LineTrackingBringupTest_RequestStart(void);
void LineTrackingBringupTest_RequestStop(void);
line_tracking_bringup_test_state_t LineTrackingBringupTest_GetState(void);
bool LineTrackingBringupTest_IsActive(void);
const char *LineTrackingBringupTest_GetStateText(void);
uint32_t LineTrackingBringupTest_GetRunCount(void);

#endif
