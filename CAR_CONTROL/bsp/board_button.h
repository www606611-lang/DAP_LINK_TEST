#ifndef BSP_BOARD_BUTTON_H
#define BSP_BOARD_BUTTON_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BOARD_BUTTON_ID_PB21 = 0,
    BOARD_BUTTON_ID_SW2_PB4,
    BOARD_BUTTON_ID_SW1_PB5,
    BOARD_BUTTON_ID_COUNT
} board_button_id_t;

void BoardButton_Init(uint32_t now_ms);
void BoardButton_Task(uint32_t now_ms);

bool BoardButton_IsPressed(void);
bool BoardButton_GetPressEvent(void);
bool BoardButton_GetReleaseEvent(void);
bool BoardButton_IsPressedId(board_button_id_t button);
bool BoardButton_GetPressEventId(board_button_id_t button);
bool BoardButton_GetReleaseEventId(board_button_id_t button);

#endif
