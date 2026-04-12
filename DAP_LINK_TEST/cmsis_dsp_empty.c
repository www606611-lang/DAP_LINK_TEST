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
#include "uart0_dma.h"
#include "timer.h"
#include "oled_status.h"

#define UART_DISPLAY_BUFFER_SIZE        32U

static uint8_t g_uart_display_buffer[UART_DISPLAY_BUFFER_SIZE];

int main(void)
{
    uint16_t uart_rx_length;

    SYSCFG_DL_init();
    timer_common_init();
    uart0_dma_init();
    uart0_dma_start_rx_stream();

    oled_status_screen_init(timer_common_get_ms());
    (void) uart0_dma_send_text("UART0 DMA OK\r\n");

    while (1) {
        uart0_dma_task();
        uart_rx_length = uart0_dma_read(g_uart_display_buffer,
            UART_DISPLAY_BUFFER_SIZE);
        if (uart_rx_length > 0U) {
            oled_status_screen_uart_write(g_uart_display_buffer,
                uart_rx_length);
        }
        oled_status_screen_task(timer_common_get_ms());
    }
}
