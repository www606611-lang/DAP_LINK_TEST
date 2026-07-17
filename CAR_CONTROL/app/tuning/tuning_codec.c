#include "tuning_codec.h"

#include <stddef.h>
#include <string.h>

uint16_t speed_tuning_tokenize(
    char *line, char **tokens, uint16_t capacity)
{
    uint16_t count = 0U;
    bool in_token = false;

    while (*line != '\0') {
        if ((*line == ' ') || (*line == '\t')) {
            *line = '\0';
            in_token = false;
        } else if (!in_token) {
            if (count >= capacity) {
                return capacity + 1U;
            }
            tokens[count++] = line;
            in_token = true;
        }
        line++;
    }
    return count;
}

bool speed_tuning_parse_float(const char *text, float *value)
{
    float parsed = 0.0f;
    float fraction_scale = 0.1f;
    bool negative = false;
    bool have_digit = false;

    if ((text == NULL) || (value == NULL)) {
        return false;
    }
    if ((*text == '+') || (*text == '-')) {
        negative = (*text == '-');
        text++;
    }
    while ((*text >= '0') && (*text <= '9')) {
        parsed = parsed * 10.0f + (float) (*text - '0');
        have_digit = true;
        text++;
    }
    if (*text == '.') {
        text++;
        while ((*text >= '0') && (*text <= '9')) {
            parsed += (float) (*text - '0') * fraction_scale;
            fraction_scale *= 0.1f;
            have_digit = true;
            text++;
        }
    }
    if (!have_digit || (*text != '\0')) {
        return false;
    }

    *value = negative ? -parsed : parsed;
    return true;
}

bool speed_tuning_parse_u16(const char *text, uint16_t *value)
{
    uint32_t parsed = 0U;
    bool have_digit = false;

    if ((text == NULL) || (value == NULL)) {
        return false;
    }
    while ((*text >= '0') && (*text <= '9')) {
        parsed = parsed * 10U + (uint32_t) (*text - '0');
        if (parsed > UINT16_MAX) {
            return false;
        }
        have_digit = true;
        text++;
    }
    if (!have_digit || (*text != '\0')) {
        return false;
    }
    *value = (uint16_t) parsed;
    return true;
}

bool speed_tuning_parse_i32(const char *text, int32_t *value)
{
    uint32_t magnitude = 0U;
    uint32_t limit = (uint32_t) INT32_MAX;
    bool negative = false;
    bool have_digit = false;

    if ((text == NULL) || (value == NULL)) {
        return false;
    }
    if ((*text == '+') || (*text == '-')) {
        negative = (*text == '-');
        if (negative) {
            limit++;
        }
        text++;
    }
    while ((*text >= '0') && (*text <= '9')) {
        uint32_t digit = (uint32_t) (*text - '0');

        if ((magnitude > (limit / 10U)) ||
            ((magnitude == (limit / 10U)) &&
                (digit > (limit % 10U)))) {
            return false;
        }
        magnitude = magnitude * 10U + digit;
        have_digit = true;
        text++;
    }
    if (!have_digit || (*text != '\0')) {
        return false;
    }

    if (negative) {
        *value = (magnitude == ((uint32_t) INT32_MAX + 1U)) ?
            INT32_MIN : -(int32_t) magnitude;
    } else {
        *value = (int32_t) magnitude;
    }
    return true;
}

bool speed_tuning_parse_profile(
    const char *text, speed_bringup_profile_t *profile)
{
    if ((text == NULL) || (profile == NULL)) {
        return false;
    }
    if (strcmp(text, "step") == 0) {
        *profile = SPEED_BRINGUP_PROFILE_STEP;
        return true;
    }
    if (strcmp(text, "reverse") == 0) {
        *profile = SPEED_BRINGUP_PROFILE_REVERSE;
        return true;
    }
    if (strcmp(text, "sweep") == 0) {
        *profile = SPEED_BRINGUP_PROFILE_SWEEP;
        return true;
    }
    if (strcmp(text, "lease") == 0) {
        *profile = SPEED_BRINGUP_PROFILE_LEASE_TEST;
        return true;
    }
    return false;
}

bool speed_tuning_parse_position_profile(
    const char *text, position_bringup_profile_t *profile)
{
    if ((text == NULL) || (profile == NULL)) {
        return false;
    }
    if (strcmp(text, "stress") == 0) {
        *profile = POSITION_BRINGUP_PROFILE_STRESS;
        return true;
    }
    return false;
}
