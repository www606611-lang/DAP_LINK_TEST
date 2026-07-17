#ifndef APP_HEADING_BRINGUP_TEST_H
#define APP_HEADING_BRINGUP_TEST_H

#include "wheel_heading_control.h"

#include <stdbool.h>
#include <stdint.h>

#define HEADING_BRINGUP_OUTPUT_MIN      100U
#define HEADING_BRINGUP_OUTPUT_MAX     1000U
#define HEADING_BRINGUP_DURATION_MIN_MS 500U
#define HEADING_BRINGUP_DURATION_MAX_MS 10000U
#define HEADING_BRINGUP_BASE_SPEED_MIN_PPS 100.0f

typedef enum {
    HEADING_BRINGUP_TEST_LOCKED = 0,
    HEADING_BRINGUP_TEST_READY,
    HEADING_BRINGUP_TEST_ARMING,
    HEADING_BRINGUP_TEST_RUNNING,
    HEADING_BRINGUP_TEST_COMPLETE,
    HEADING_BRINGUP_TEST_ABORTED
} heading_bringup_test_state_t;

typedef enum {
    HEADING_BRINGUP_CONFIG_OK = 0,
    HEADING_BRINGUP_CONFIG_BAD_ARGUMENT,
    HEADING_BRINGUP_CONFIG_BAD_RANGE,
    HEADING_BRINGUP_CONFIG_BUSY
} heading_bringup_config_result_t;

typedef struct {
    wheel_heading_control_config_t control;
    float base_speed_pps;
    uint16_t output_limit_permille;
    uint32_t duration_ms;
} heading_bringup_config_t;

void HeadingBringupTest_Init(bool reset_locked);
void HeadingBringupTest_Task(uint32_t now_ms);
heading_bringup_config_result_t HeadingBringupTest_SetConfig(
    const heading_bringup_config_t *config);
bool HeadingBringupTest_GetConfig(heading_bringup_config_t *config);
bool HeadingBringupTest_RequestStart(void);
void HeadingBringupTest_RequestStop(void);
heading_bringup_test_state_t HeadingBringupTest_GetState(void);
bool HeadingBringupTest_IsActive(void);
const char *HeadingBringupTest_GetStateText(void);
uint32_t HeadingBringupTest_GetRunCount(void);

#endif
