// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2016 Scott Shawcroft
//
// SPDX-License-Identifier: MIT

#pragma once

#include "common-hal/microcontroller/Pin.h"

#include "hal/uart_types.h"
#include "py/obj.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

typedef struct {
    mp_obj_base_t base;
    const mcu_pin_obj_t *rx_pin;
    const mcu_pin_obj_t *tx_pin;
    const mcu_pin_obj_t *rts_pin;
    const mcu_pin_obj_t *cts_pin;
    uart_port_t uart_num;
    uint8_t character_bits;
    bool rx_error;
    uint32_t timeout_ms;
    bool is_console;
    QueueHandle_t event_queue;
    TaskHandle_t event_task;
} busio_uart_obj_t;

typedef struct uart_transfer_state uart_transfer_state;

#if CIRCUITPY_BUSIO_NOBLOCK
uart_transfer_state *common_hal_busio_uart_start_write(busio_uart_obj_t *uart, const uint8_t *data, size_t len);
bool common_hal_busio_uart_write_isbusy(uart_transfer_state *state);
#endif

void uart_reset(void);
