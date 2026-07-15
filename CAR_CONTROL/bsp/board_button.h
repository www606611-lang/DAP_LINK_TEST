#ifndef BSP_BOARD_BUTTON_H
#define BSP_BOARD_BUTTON_H

#include <stdbool.h>
#include <stdint.h>

void BoardButton_Init(uint32_t now_ms);
void BoardButton_Task(uint32_t now_ms);

bool BoardButton_IsPressed(void);
bool BoardButton_GetPressEvent(void);
bool BoardButton_GetReleaseEvent(void);

#endif
