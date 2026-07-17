#include "line_tracking_finish_policy.h"

#include <stddef.h>

static bool line_tracking_finish_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t) (now_ms - deadline_ms) >= 0);
}

void LineTrackingFinishPolicy_Init(
    line_tracking_finish_policy_t *policy,
    uint32_t minimum_deadline_ms, uint32_t grace_ms)
{
    if (policy == NULL) {
        return;
    }
    policy->minimum_deadline_ms = minimum_deadline_ms;
    policy->grace_deadline_ms = minimum_deadline_ms + grace_ms;
    policy->centered_since_ms = minimum_deadline_ms;
    policy->centered_pending = false;
}

line_tracking_finish_decision_t LineTrackingFinishPolicy_Update(
    line_tracking_finish_policy_t *policy,
    uint32_t now_ms, bool centered, uint32_t stable_ms)
{
    if (policy == NULL) {
        return LINE_TRACKING_FINISH_TIMEOUT;
    }
    if (!line_tracking_finish_deadline_reached(
            now_ms, policy->minimum_deadline_ms)) {
        return LINE_TRACKING_FINISH_CONTINUE;
    }

    if (centered) {
        if (!policy->centered_pending) {
            policy->centered_pending = true;
            policy->centered_since_ms = now_ms;
        } else if ((uint32_t) (now_ms - policy->centered_since_ms) >=
            stable_ms) {
            return LINE_TRACKING_FINISH_COMPLETE;
        }
    } else {
        policy->centered_pending = false;
    }

    if (line_tracking_finish_deadline_reached(
            now_ms, policy->grace_deadline_ms)) {
        return LINE_TRACKING_FINISH_TIMEOUT;
    }
    return LINE_TRACKING_FINISH_CONTINUE;
}
