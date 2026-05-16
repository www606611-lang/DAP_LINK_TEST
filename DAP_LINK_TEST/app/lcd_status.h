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
void lcd_status_screen_set_line_sensor(uint8_t raw, uint8_t active_mask,
    uint8_t active_count, int16_t line_error, uint8_t enabled,
    uint8_t sensor_ok, uint8_t sensor_error);

#ifdef __cplusplus
}
#endif

#endif
