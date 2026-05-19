#ifndef MODULES_KEY_H
#define MODULES_KEY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    KEY_ID_B21 = 0,
    KEY_ID_DOWN,
    KEY_ID_COUNT
} key_id_t;

void Key_Init(uint32_t now_ms);
void Key_Task(uint32_t now_ms);

bool Key_IsPressed(key_id_t key);
bool Key_GetPressEvent(key_id_t key);
bool Key_GetReleaseEvent(key_id_t key);

#ifdef __cplusplus
}
#endif

#endif
