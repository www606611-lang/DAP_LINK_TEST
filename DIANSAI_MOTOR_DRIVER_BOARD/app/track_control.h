#ifndef APP_TRACK_CONTROL_H
#define APP_TRACK_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void track_control_init(uint32_t now_ms);
void track_control_task(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
