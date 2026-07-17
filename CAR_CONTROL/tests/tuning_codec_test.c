#include "tuning_codec.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void test_tokenize(void)
{
    char line[] = "  yaw\tset  45.0 ";
    char overflow[] = "a b c";
    char *tokens[4];
    uint16_t count;

    count = speed_tuning_tokenize(line, tokens, 4U);
    assert(count == 3U);
    assert(strcmp(tokens[0], "yaw") == 0);
    assert(strcmp(tokens[1], "set") == 0);
    assert(strcmp(tokens[2], "45.0") == 0);

    count = speed_tuning_tokenize(overflow, tokens, 2U);
    assert(count == 3U);
}

static void test_float_parser(void)
{
    float value;

    assert(speed_tuning_parse_float("-45.125", &value));
    assert(abs_float(value + 45.125f) < 0.0001f);
    assert(speed_tuning_parse_float(".5", &value));
    assert(abs_float(value - 0.5f) < 0.0001f);
    assert(speed_tuning_parse_float("1.", &value));
    assert(abs_float(value - 1.0f) < 0.0001f);
    assert(!speed_tuning_parse_float("", &value));
    assert(!speed_tuning_parse_float("-", &value));
    assert(!speed_tuning_parse_float("1e3", &value));
    assert(!speed_tuning_parse_float("1.2.3", &value));
    assert(!speed_tuning_parse_float(NULL, &value));
    assert(!speed_tuning_parse_float("1", NULL));
}

static void test_integer_parsers(void)
{
    uint16_t u16;
    int32_t i32;

    assert(speed_tuning_parse_u16("0", &u16) && (u16 == 0U));
    assert(speed_tuning_parse_u16("65535", &u16) &&
        (u16 == UINT16_MAX));
    assert(!speed_tuning_parse_u16("65536", &u16));
    assert(!speed_tuning_parse_u16("-1", &u16));
    assert(!speed_tuning_parse_u16("1.0", &u16));

    assert(speed_tuning_parse_i32("2147483647", &i32) &&
        (i32 == INT32_MAX));
    assert(speed_tuning_parse_i32("-2147483648", &i32) &&
        (i32 == INT32_MIN));
    assert(speed_tuning_parse_i32("+42", &i32) && (i32 == 42));
    assert(!speed_tuning_parse_i32("2147483648", &i32));
    assert(!speed_tuning_parse_i32("-2147483649", &i32));
    assert(!speed_tuning_parse_i32("+", &i32));
}

static void test_profiles(void)
{
    speed_bringup_profile_t speed_profile;
    position_bringup_profile_t position_profile;

    assert(speed_tuning_parse_profile("step", &speed_profile));
    assert(speed_profile == SPEED_BRINGUP_PROFILE_STEP);
    assert(speed_tuning_parse_profile("reverse", &speed_profile));
    assert(speed_profile == SPEED_BRINGUP_PROFILE_REVERSE);
    assert(speed_tuning_parse_profile("sweep", &speed_profile));
    assert(speed_profile == SPEED_BRINGUP_PROFILE_SWEEP);
    assert(speed_tuning_parse_profile("lease", &speed_profile));
    assert(speed_profile == SPEED_BRINGUP_PROFILE_LEASE_TEST);
    assert(!speed_tuning_parse_profile("ramp", &speed_profile));

    assert(speed_tuning_parse_position_profile(
        "stress", &position_profile));
    assert(position_profile == POSITION_BRINGUP_PROFILE_STRESS);
    assert(!speed_tuning_parse_position_profile(
        "single", &position_profile));
}

int main(void)
{
    test_tokenize();
    test_float_parser();
    test_integer_parsers();
    test_profiles();
    return 0;
}
