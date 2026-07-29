#include "h_route_events.h"

#include <assert.h>
#include <string.h>

static h_route_config_t valid_config(void)
{
    h_route_config_t config = {
        20, 80, 500, 900, 25, 6U, 20U, 20U
    };
    return config;
}

static h_route_input_t route_input(uint32_t now_ms,
    int32_t count, uint8_t active_count)
{
    h_route_input_t input;

    memset(&input, 0, sizeof(input));
    input.now_ms = now_ms;
    input.odometry_count = count;
    input.line_active_count = active_count;
    input.odometry_ready = true;
    input.line_sensor_ready = true;
    return input;
}

static void update(h_route_events_t *events, uint32_t now_ms,
    int32_t count, uint8_t active_count)
{
    h_route_input_t input = route_input(now_ms, count, active_count);
    HRouteEvents_Update(events, &input);
}

static void confirm_initial_a_and_leave(h_route_events_t *events)
{
    update(events, 0U, 0, 6U);
    update(events, 20U, 0, 6U);
    assert(events->snapshot.initial_a_seen);
    assert(events->snapshot.marker_wide);

    update(events, 30U, 10, 2U);
    update(events, 50U, 25, 2U);
    assert(events->snapshot.left_start_a_event);
    assert(events->snapshot.left_start_a);
}

static void test_invalid_config_blocks_start(void)
{
    h_route_events_t events;
    h_route_config_t config = {0};

    HRouteEvents_Init(&events, &config);
    assert(!events.snapshot.configuration_valid);
    assert(!HRouteEvents_Start(&events, 0U, 0));
}

static void test_initial_a_needs_debounce_and_release(void)
{
    h_route_events_t events;
    h_route_config_t config = valid_config();

    HRouteEvents_Init(&events, &config);
    assert(HRouteEvents_Start(&events, 0U, 100));
    update(&events, 0U, 100, 6U);
    update(&events, 19U, 100, 6U);
    assert(!events.snapshot.initial_a_seen);
    update(&events, 20U, 100, 6U);
    assert(events.snapshot.initial_a_seen);

    update(&events, 30U, 130, 2U);
    update(&events, 49U, 130, 2U);
    assert(!events.snapshot.left_start_a);
    update(&events, 50U, 130, 2U);
    assert(events.snapshot.left_start_a_event);
    update(&events, 51U, 131, 2U);
    assert(!events.snapshot.left_start_a_event);
}

static void test_b_and_finish_require_ordered_evidence(void)
{
    h_route_events_t events;
    h_route_config_t config = valid_config();

    HRouteEvents_Init(&events, &config);
    assert(HRouteEvents_Start(&events, 0U, 0));
    confirm_initial_a_and_leave(&events);

    update(&events, 100U, 400, 6U);
    update(&events, 120U, 400, 6U);
    assert(!events.snapshot.finish_a_passed);
    update(&events, 130U, 450, 2U);
    update(&events, 150U, 450, 2U);
    update(&events, 160U, 500, 2U);
    assert(events.snapshot.b_passed_event);
    assert(!events.snapshot.finish_a_passed);

    update(&events, 170U, 899, 6U);
    update(&events, 189U, 899, 6U);
    assert(!events.snapshot.finish_a_passed);
    update(&events, 190U, 900, 6U);
    assert(events.snapshot.finish_a_passed_event);
    update(&events, 171U, 901, 6U);
    assert(!events.snapshot.finish_a_passed_event);
}

static void test_missing_initial_a_never_arms_lap(void)
{
    h_route_events_t events;
    h_route_config_t config = valid_config();

    HRouteEvents_Init(&events, &config);
    assert(HRouteEvents_Start(&events, 0U, 0));
    update(&events, 0U, 100, 2U);
    update(&events, 100U, 500, 2U);
    assert(!events.snapshot.initial_a_seen);
    assert(!events.snapshot.left_start_a);
    assert(!events.snapshot.b_passed);
}

static void test_old_wide_marker_cannot_finish_after_threshold(void)
{
    h_route_events_t events;
    h_route_config_t config = valid_config();

    HRouteEvents_Init(&events, &config);
    assert(HRouteEvents_Start(&events, 0U, 0));
    confirm_initial_a_and_leave(&events);
    update(&events, 60U, 500, 2U);
    assert(events.snapshot.b_passed);

    update(&events, 70U, 850, 6U);
    update(&events, 90U, 850, 6U);
    update(&events, 100U, 900, 6U);
    assert(!events.snapshot.finish_a_passed);

    update(&events, 110U, 920, 2U);
    update(&events, 130U, 920, 2U);
    update(&events, 140U, 930, 6U);
    update(&events, 160U, 930, 6U);
    assert(events.snapshot.finish_a_passed_event);
}

static void test_invalid_input_suppresses_events(void)
{
    h_route_events_t events;
    h_route_config_t config = valid_config();
    h_route_input_t input = route_input(0U, 0, 6U);

    HRouteEvents_Init(&events, &config);
    assert(HRouteEvents_Start(&events, 0U, 0));
    input.line_sensor_ready = false;
    HRouteEvents_Update(&events, &input);
    input.now_ms = 100U;
    HRouteEvents_Update(&events, &input);
    assert(!events.snapshot.input_valid);
    assert(!events.snapshot.initial_a_seen);

    input.line_sensor_ready = true;
    HRouteEvents_Update(&events, &input);
    input.now_ms = 119U;
    HRouteEvents_Update(&events, &input);
    assert(!events.snapshot.initial_a_seen);
    input.now_ms = 120U;
    HRouteEvents_Update(&events, &input);
    assert(events.snapshot.initial_a_seen);
}

int main(void)
{
    test_invalid_config_blocks_start();
    test_initial_a_needs_debounce_and_release();
    test_b_and_finish_require_ordered_evidence();
    test_missing_initial_a_never_arms_lap();
    test_old_wide_marker_cannot_finish_after_threshold();
    test_invalid_input_suppresses_events();
    return 0;
}
