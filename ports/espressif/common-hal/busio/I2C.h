// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2016 Scott Shawcroft
//
// SPDX-License-Identifier: MIT

#pragma once

#include "common-hal/microcontroller/Pin.h"

#include "components/hal/include/hal/i2c_types.h"
#include "FreeRTOS.h"
#include "freertos/semphr.h"
#include "py/obj.h"

#include "driver/i2c_master.h"

typedef struct {
    mp_obj_base_t base;
    const mcu_pin_obj_t *scl_pin;
    const mcu_pin_obj_t *sda_pin;
    size_t timeout_ms;
    size_t frequency;
    i2c_master_bus_handle_t handle;
    SemaphoreHandle_t xSemaphore;
    bool has_lock;
} busio_i2c_obj_t;

typedef struct i2c_transfer_state i2c_transfer_state;

#if CIRCUITPY_BUSIO_NOBLOCK
i2c_transfer_state *common_hal_busio_i2c_start_read(busio_i2c_obj_t *i2c, uint8_t address, uint8_t *data, size_t len, bool nostop);
i2c_transfer_state *common_hal_busio_i2c_start_write(busio_i2c_obj_t *i2c, uint8_t address, const uint8_t *data, size_t len, bool nostop);
bool common_hal_busio_i2c_read_isbusy(i2c_transfer_state *state);
bool common_hal_busio_i2c_write_isbusy(i2c_transfer_state *state);
#endif
