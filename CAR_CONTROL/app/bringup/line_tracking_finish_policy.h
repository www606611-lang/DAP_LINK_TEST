#ifndef APP_LINE_TRACKING_FINISH_POLICY_H
#define APP_LINE_TRACKING_FINISH_POLICY_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LINE_TRACKING_FINISH_CONTINUE = 0,
    LINE_TRACKING_FINISH_COMPLETE,
    LINE_TRACKING_FINISH_TIMEOUT
} line_tracking_finish_decision_t;

typedef struct {
    uint32_t minimum_deadline_ms;
    uint32_t grace_deadline_ms;
    uint32_t centered_since_ms;
    bool centered_pending;
} line_tracking_finish_policy_t;

void LineTrackingFinishPolicy_Init(
    line_tracking_finish_policy_t *policy,
    uint32_t minimum_deadline_ms, uint32_t grace_ms);
line_tracking_finish_decision_t LineTrackingFinishPolicy_Update(
    line_tracking_finish_policy_t *policy,
    uint32_t now_ms, bool centered, uint32_t stable_ms);

#endif
