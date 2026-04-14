#ifndef APP_LCD_STATUS_H
#define APP_LCD_STATUS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void lcd_status_screen_init(uint32_t now_ms);
void lcd_status_screen_task(uint32_t now_ms);
void lcd_status_screen_uart_put(uint8_t data);
void lcd_status_screen_uart_write(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
