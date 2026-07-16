#ifndef APP_SPEED_BRINGUP_TEST_H
#define APP_SPEED_BRINGUP_TEST_H

#include <stdbool.h>
#include <stdint.h>

#include "wheel_speed_control.h"

#define SPEED_BRINGUP_TARGET_MIN_PPS   100.0f
#define SPEED_BRINGUP_TARGET_MAX_PPS   WHEEL_SPEED_CONTROL_TARGET_MAX_PPS
#define SPEED_BRINGUP_OUTPUT_MIN       100U
#define SPEED_BRINGUP_OUTPUT_MAX       1000U

typedef enum {
    SPEED_BRINGUP_TEST_LOCKED = 0,
    SPEED_BRINGUP_TEST_READY,
    SPEED_BRINGUP_TEST_RUNNING,
    SPEED_BRINGUP_TEST_COMPLETE,
    SPEED_BRINGUP_TEST_ABORTED
} speed_bringup_test_state_t;

typedef enum {
    SPEED_BRINGUP_PROFILE_RAMP = 0,
    SPEED_BRINGUP_PROFILE_STEP,
    SPEED_BRINGUP_PROFILE_REVERSE,
    SPEED_BRINGUP_PROFILE_SWEEP
} speed_bringup_profile_t;

typedef enum {
    SPEED_BRINGUP_CONFIG_OK = 0,
    SPEED_BRINGUP_CONFIG_BAD_ARGUMENT,
    SPEED_BRINGUP_CONFIG_BAD_RANGE,
    SPEED_BRINGUP_CONFIG_BUSY
} speed_bringup_config_result_t;

typedef struct {
    wheel_speed_control_tunings_t pid;
    float target_pps;
    uint16_t output_limit_permille;
} speed_bringup_config_t;

void SpeedBringupTest_Init(bool reset_locked);
void SpeedBringupTest_Task(uint32_t now_ms, bool press_event);
speed_bringup_config_result_t SpeedBringupTest_SetConfig(
    const speed_bringup_config_t *config);
bool SpeedBringupTest_GetConfig(speed_bringup_config_t *config);
bool SpeedBringupTest_RequestStart(void);
bool SpeedBringupTest_RequestProfile(speed_bringup_profile_t profile);
void SpeedBringupTest_RequestStop(void);
speed_bringup_test_state_t SpeedBringupTest_GetState(void);
const char *SpeedBringupTest_GetStateText(void);
speed_bringup_profile_t SpeedBringupTest_GetProfile(void);
const char *SpeedBringupTest_GetProfileText(void);
uint32_t SpeedBringupTest_GetRunCount(void);

#endif
