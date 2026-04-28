#ifndef APP_LCD_STATUS_H
#define APP_LCD_STATUS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* LCD 状态页：显示运行时间、串口最近一帧、编码器和 IMU 数据。 */
void lcd_status_screen_init(uint32_t now_ms);
void lcd_status_screen_task(uint32_t now_ms);
void lcd_status_screen_uart_put(uint8_t data);
void lcd_status_screen_uart_write(const uint8_t *data, uint16_t length);
void lcd_status_screen_set_k230(
    uint8_t valid, uint16_t cx, uint16_t cy, int16_t err_x, int16_t err_y);

#ifdef __cplusplus
}
#endif

#endif
