#ifndef APP_ENCODER_SPEED_TEST_H
#define APP_ENCODER_SPEED_TEST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void encoder_speed_test_init(uint32_t now_ms);
void encoder_speed_test_task(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
