#include "heading_bringup_test.h"
#include "line_tracking_bringup_test.h"
#include "position_bringup_test.h"
#include "speed_bringup_test.h"
#include "yaw_bringup_test.h"

#include <stddef.h>

void SpeedBringupTest_Init(bool reset_locked)
{
    (void) reset_locked;
}

void SpeedBringupTest_Task(uint32_t now_ms, bool press_event)
{
    (void) now_ms;
    (void) press_event;
}

speed_bringup_config_result_t SpeedBringupTest_SetConfig(
    const speed_bringup_config_t *config)
{
    (void) config;
    return SPEED_BRINGUP_CONFIG_BUSY;
}

bool SpeedBringupTest_GetConfig(speed_bringup_config_t *config)
{
    (void) config;
    return false;
}

bool SpeedBringupTest_RequestStart(void)
{
    return false;
}

bool SpeedBringupTest_RequestProfile(speed_bringup_profile_t profile)
{
    (void) profile;
    return false;
}

void SpeedBringupTest_RequestStop(void)
{
}

speed_bringup_test_state_t SpeedBringupTest_GetState(void)
{
    return SPEED_BRINGUP_TEST_LOCKED;
}

const char *SpeedBringupTest_GetStateText(void)
{
    return "DISABLED";
}

speed_bringup_profile_t SpeedBringupTest_GetProfile(void)
{
    return SPEED_BRINGUP_PROFILE_RAMP;
}

const char *SpeedBringupTest_GetProfileText(void)
{
    return "DISABLED";
}

uint32_t SpeedBringupTest_GetRunCount(void)
{
    return 0U;
}

void PositionBringupTest_Init(bool reset_locked)
{
    (void) reset_locked;
}

void PositionBringupTest_Task(uint32_t now_ms, bool press_event)
{
    (void) now_ms;
    (void) press_event;
}

position_bringup_config_result_t PositionBringupTest_SetConfig(
    const position_bringup_config_t *config)
{
    (void) config;
    return POSITION_BRINGUP_CONFIG_BUSY;
}

bool PositionBringupTest_GetConfig(position_bringup_config_t *config)
{
    (void) config;
    return false;
}

bool PositionBringupTest_RequestStart(void)
{
    return false;
}

bool PositionBringupTest_RequestProfile(
    position_bringup_profile_t profile)
{
    (void) profile;
    return false;
}

bool PositionBringupTest_RequestMove(int32_t delta_counts)
{
    (void) delta_counts;
    return false;
}

void PositionBringupTest_RequestStop(void)
{
}

position_bringup_test_state_t PositionBringupTest_GetState(void)
{
    return POSITION_BRINGUP_TEST_LOCKED;
}

const char *PositionBringupTest_GetStateText(void)
{
    return "DISABLED";
}

position_bringup_profile_t PositionBringupTest_GetProfile(void)
{
    return POSITION_BRINGUP_PROFILE_SINGLE;
}

const char *PositionBringupTest_GetProfileText(void)
{
    return "DISABLED";
}

uint8_t PositionBringupTest_GetCurrentStep(void)
{
    return 0U;
}

uint8_t PositionBringupTest_GetStepCount(void)
{
    return 0U;
}

uint8_t PositionBringupTest_GetCompletedMoveCount(void)
{
    return 0U;
}

uint32_t PositionBringupTest_GetWorstFinalErrorCount(void)
{
    return 0U;
}

uint32_t PositionBringupTest_GetLeftRecoveryCount(void)
{
    return 0U;
}

uint32_t PositionBringupTest_GetRightRecoveryCount(void)
{
    return 0U;
}

uint32_t PositionBringupTest_GetRunCount(void)
{
    return 0U;
}

void HeadingBringupTest_Init(bool reset_locked)
{
    (void) reset_locked;
}

void HeadingBringupTest_Task(uint32_t now_ms)
{
    (void) now_ms;
}

heading_bringup_config_result_t HeadingBringupTest_SetConfig(
    const heading_bringup_config_t *config)
{
    (void) config;
    return HEADING_BRINGUP_CONFIG_BUSY;
}

bool HeadingBringupTest_GetConfig(heading_bringup_config_t *config)
{
    (void) config;
    return false;
}

bool HeadingBringupTest_RequestStart(void)
{
    return false;
}

void HeadingBringupTest_RequestStop(void)
{
}

heading_bringup_test_state_t HeadingBringupTest_GetState(void)
{
    return HEADING_BRINGUP_TEST_LOCKED;
}

bool HeadingBringupTest_IsActive(void)
{
    return false;
}

const char *HeadingBringupTest_GetStateText(void)
{
    return "DISABLED";
}

uint32_t HeadingBringupTest_GetRunCount(void)
{
    return 0U;
}

void LineTrackingBringupTest_Init(bool reset_locked)
{
    (void) reset_locked;
}

void LineTrackingBringupTest_Task(uint32_t now_ms)
{
    (void) now_ms;
}

line_tracking_bringup_config_result_t LineTrackingBringupTest_SetConfig(
    const line_tracking_bringup_config_t *config)
{
    (void) config;
    return LINE_TRACKING_BRINGUP_CONFIG_BUSY;
}

bool LineTrackingBringupTest_GetConfig(
    line_tracking_bringup_config_t *config)
{
    (void) config;
    return false;
}

bool LineTrackingBringupTest_RequestStart(void)
{
    return false;
}

void LineTrackingBringupTest_RequestStop(void)
{
}

line_tracking_bringup_test_state_t LineTrackingBringupTest_GetState(void)
{
    return LINE_TRACKING_BRINGUP_TEST_LOCKED;
}

bool LineTrackingBringupTest_IsActive(void)
{
    return false;
}

const char *LineTrackingBringupTest_GetStateText(void)
{
    return "DISABLED";
}

uint32_t LineTrackingBringupTest_GetRunCount(void)
{
    return 0U;
}

void YawBringupTest_Init(bool reset_locked)
{
    (void) reset_locked;
}

void YawBringupTest_Task(uint32_t now_ms)
{
    (void) now_ms;
}

yaw_bringup_config_result_t YawBringupTest_SetConfig(
    const yaw_bringup_config_t *config)
{
    (void) config;
    return YAW_BRINGUP_CONFIG_BUSY;
}

bool YawBringupTest_GetConfig(yaw_bringup_config_t *config)
{
    (void) config;
    return false;
}

bool YawBringupTest_RequestStart(void)
{
    return false;
}

bool YawBringupTest_RequestTurn(float delta_yaw_deg)
{
    (void) delta_yaw_deg;
    return false;
}

void YawBringupTest_RequestStop(void)
{
}

yaw_bringup_test_state_t YawBringupTest_GetState(void)
{
    return YAW_BRINGUP_TEST_LOCKED;
}

bool YawBringupTest_IsActive(void)
{
    return false;
}

const char *YawBringupTest_GetStateText(void)
{
    return "DISABLED";
}

uint32_t YawBringupTest_GetRunCount(void)
{
    return 0U;
}
