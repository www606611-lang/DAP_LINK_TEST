#ifndef APP_TUNING_TUNING_CODEC_H
#define APP_TUNING_TUNING_CODEC_H

#include "position_bringup_test.h"
#include "speed_bringup_test.h"

#include <stdbool.h>
#include <stdint.h>

uint16_t speed_tuning_tokenize(
    char *line, char **tokens, uint16_t capacity);
bool speed_tuning_parse_float(const char *text, float *value);
bool speed_tuning_parse_i32(const char *text, int32_t *value);
bool speed_tuning_parse_u16(const char *text, uint16_t *value);
bool speed_tuning_parse_profile(
    const char *text, speed_bringup_profile_t *profile);
bool speed_tuning_parse_position_profile(
    const char *text, position_bringup_profile_t *profile);

#endif
