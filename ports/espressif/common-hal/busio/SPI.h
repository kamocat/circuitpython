// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2021 microDev
//
// SPDX-License-Identifier: MIT

#pragma once

#include "driver/spi_master.h"
#include "shared-bindings/microcontroller/Pin.h"

typedef struct {
    mp_obj_base_t base;

    const mcu_pin_obj_t *MOSI;
    const mcu_pin_obj_t *MISO;
    const mcu_pin_obj_t *clock;

    spi_host_device_t host_id;

    uint8_t bits;
    uint8_t phase;
    uint8_t polarity;
    uint32_t baudrate;

    SemaphoreHandle_t mutex;
} busio_spi_obj_t;

typedef struct spi_transfer_state spi_transfer_state;

#if CIRCUITPY_BUSIO_NOBLOCK
spi_transfer_state *common_hal_busio_spi_start_transfer(busio_spi_obj_t *spi, const uint8_t *out_data, uint8_t *in_data, size_t len);
bool common_hal_busio_spi_transfer_isbusy(spi_transfer_state *state);
#endif
