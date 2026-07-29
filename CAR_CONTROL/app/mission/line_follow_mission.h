#ifndef APP_MISSION_LINE_FOLLOW_MISSION_H
#define APP_MISSION_LINE_FOLLOW_MISSION_H

#include "wheel_line_tracking_control.h"

#include <stdbool.h>
#include <stdint.h>

#define LINE_FOLLOW_MISSION_BASE_SPEED_PPS          1400.0f
#define LINE_FOLLOW_MISSION_OUTPUT_LIMIT_PERMILLE     750U

typedef enum {
    LINE_FOLLOW_MISSION_LOCKED = 0,
    LINE_FOLLOW_MISSION_READY,
    LINE_FOLLOW_MISSION_RUNNING,
    LINE_FOLLOW_MISSION_STOPPED,
    LINE_FOLLOW_MISSION_FAULT
} line_follow_mission_state_t;

typedef struct {
    line_follow_mission_state_t state;
    wheel_line_tracking_result_t last_result;
    float base_speed_pps;
    uint16_t output_limit_permille;
    uint32_t run_count;
    uint32_t elapsed_ms;
    bool centered_start_active;
} line_follow_mission_snapshot_t;

void LineFollowMission_Init(bool reset_locked);
void LineFollowMission_Task(uint32_t now_ms);
bool LineFollowMission_RequestStart(void);
bool LineFollowMission_RequestStartFromWideMarker(
    uint8_t narrow_active_max);
bool LineFollowMission_SetBaseSpeed(float base_speed_pps);
void LineFollowMission_RequestStop(void);
bool LineFollowMission_IsActive(void);
const char *LineFollowMission_GetStateText(void);
bool LineFollowMission_GetSnapshot(
    line_follow_mission_snapshot_t *snapshot);

#endif
