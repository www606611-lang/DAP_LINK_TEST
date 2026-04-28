/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "timer.h"
#include "encoder.h"
#include "icm20948.h"
#include "k230_uart.h"
#include "lcd_status.h"
#include "motor.h"
#include "track_control.h"
#include "uart_display.h"

static void app_init(void);
static void app_task(void);

int main(void)
{
    app_init();

    while (1) {
        app_task();
    }
}

static void app_init(void)
{
    uint32_t now_ms;

    SYSCFG_DL_init();
    Motor_Init();
    timer_common_init();

    now_ms = timer_common_get_ms();
    track_control_init(now_ms);
    Encoder_Init(now_ms);
    ICM20948_TaskInit(now_ms);
    lcd_status_screen_init(timer_common_get_ms());
    uart_display_init();
    k230_uart_init();
}

static void app_task(void)
{
    uint32_t now_ms = timer_common_get_ms();

    Encoder_Task(now_ms);
    ICM20948_Task(now_ms);
    uart_display_task(now_ms);
    k230_uart_task(now_ms);
    track_control_task(now_ms);
    lcd_status_screen_task(now_ms);
}
