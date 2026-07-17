#include "line_tracking_finish_policy.h"

#include <assert.h>
#include <stdint.h>

static void test_stable_center_completes(void)
{
    line_tracking_finish_policy_t policy;

    LineTrackingFinishPolicy_Init(&policy, 1000U, 3000U);
    assert(LineTrackingFinishPolicy_Update(
        &policy, 999U, true, 250U) == LINE_TRACKING_FINISH_CONTINUE);
    assert(LineTrackingFinishPolicy_Update(
        &policy, 1000U, true, 250U) == LINE_TRACKING_FINISH_CONTINUE);
    assert(LineTrackingFinishPolicy_Update(
        &policy, 1249U, true, 250U) == LINE_TRACKING_FINISH_CONTINUE);
    assert(LineTrackingFinishPolicy_Update(
        &policy, 1250U, true, 250U) == LINE_TRACKING_FINISH_COMPLETE);
}

static void test_center_loss_restarts_stability(void)
{
    line_tracking_finish_policy_t policy;

    LineTrackingFinishPolicy_Init(&policy, 1000U, 3000U);
    assert(LineTrackingFinishPolicy_Update(
        &policy, 1000U, true, 250U) == LINE_TRACKING_FINISH_CONTINUE);
    assert(LineTrackingFinishPolicy_Update(
        &policy, 1100U, false, 250U) == LINE_TRACKING_FINISH_CONTINUE);
    assert(LineTrackingFinishPolicy_Update(
        &policy, 1200U, true, 250U) == LINE_TRACKING_FINISH_CONTINUE);
    assert(LineTrackingFinishPolicy_Update(
        &policy, 1449U, true, 250U) == LINE_TRACKING_FINISH_CONTINUE);
    assert(LineTrackingFinishPolicy_Update(
        &policy, 1450U, true, 250U) == LINE_TRACKING_FINISH_COMPLETE);
}

static void test_grace_expiry_is_not_success(void)
{
    line_tracking_finish_policy_t policy;

    LineTrackingFinishPolicy_Init(&policy, 1000U, 3000U);
    assert(LineTrackingFinishPolicy_Update(
        &policy, 3999U, false, 250U) == LINE_TRACKING_FINISH_CONTINUE);
    assert(LineTrackingFinishPolicy_Update(
        &policy, 4000U, false, 250U) == LINE_TRACKING_FINISH_TIMEOUT);
}

static void test_deadlines_survive_u32_wrap(void)
{
    line_tracking_finish_policy_t policy;
    const uint32_t minimum_deadline = UINT32_MAX - 50U;

    LineTrackingFinishPolicy_Init(&policy, minimum_deadline, 300U);
    assert(LineTrackingFinishPolicy_Update(
        &policy, minimum_deadline - 1U, false, 20U) ==
        LINE_TRACKING_FINISH_CONTINUE);
    assert(LineTrackingFinishPolicy_Update(
        &policy, minimum_deadline, true, 20U) ==
        LINE_TRACKING_FINISH_CONTINUE);
    assert(LineTrackingFinishPolicy_Update(
        &policy, minimum_deadline + 20U, true, 20U) ==
        LINE_TRACKING_FINISH_COMPLETE);

    LineTrackingFinishPolicy_Init(&policy, minimum_deadline, 300U);
    assert(LineTrackingFinishPolicy_Update(
        &policy, 248U, false, 20U) == LINE_TRACKING_FINISH_CONTINUE);
    assert(LineTrackingFinishPolicy_Update(
        &policy, 249U, false, 20U) == LINE_TRACKING_FINISH_TIMEOUT);
}

int main(void)
{
    test_stable_center_completes();
    test_center_loss_restarts_stability();
    test_grace_expiry_is_not_success();
    test_deadlines_survive_u32_wrap();
    return 0;
}
