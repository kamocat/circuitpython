// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2026
//
// SPDX-License-Identifier: MIT

#include "py/mperrno.h"
#include "py/runtime.h"
#include "py/stream.h"

#include "shared-bindings/busio/dma.h"
#include "shared-bindings/busio/I2C.h"
#include "shared-bindings/busio/SPI.h"
#include "shared-bindings/busio/UART.h"

#if CIRCUITPY_BUSIO_DMA

enum {
    BUSIO_DMA_I2C_CHANNEL = 0,
    BUSIO_DMA_SPI_CHANNEL = 1,
    BUSIO_DMA_UART_CHANNEL = 2,
};

uint common_hal_busio_dma_i2c_read(busio_i2c_obj_t *i2c, uint8_t address, uint8_t *data, size_t len, bool nostop) {
    if (nostop) {
        mp_raise_NotImplementedError(MP_ERROR_TEXT("no-stop read not supported"));
    }

    mp_negative_errno_t err = common_hal_busio_i2c_read(i2c, address, data, len);
    if (err < 0) {
        mp_raise_OSError(-err);
    }

    return BUSIO_DMA_I2C_CHANNEL;
}

uint common_hal_busio_dma_i2c_write(busio_i2c_obj_t *i2c, uint8_t address, const uint8_t *data, size_t len, bool nostop) {
    if (nostop) {
        mp_raise_NotImplementedError(MP_ERROR_TEXT("no-stop write not supported"));
    }

    mp_negative_errno_t err = common_hal_busio_i2c_write(i2c, address, data, len);
    if (err < 0) {
        mp_raise_OSError(-err);
    }

    return BUSIO_DMA_I2C_CHANNEL;
}

bool common_hal_busio_dma_i2c_is_busy(uint dma_channel) {
    (void)dma_channel;
    return false;
}

uint common_hal_busio_dma_spi_write(busio_spi_obj_t *spi, const uint8_t *data, size_t len) {
    if (!common_hal_busio_spi_write(spi, data, len)) {
        mp_raise_OSError(MP_EIO);
    }

    return BUSIO_DMA_SPI_CHANNEL;
}

uint common_hal_busio_dma_spi_read(busio_spi_obj_t *spi, uint8_t write_value, uint8_t *data, size_t len) {
    if (!common_hal_busio_spi_read(spi, data, len, write_value)) {
        mp_raise_OSError(MP_EIO);
    }

    return BUSIO_DMA_SPI_CHANNEL;
}

uint common_hal_busio_dma_spi_transfer(busio_spi_obj_t *spi, const uint8_t *out_data, uint8_t *in_data, size_t len) {
    if (!common_hal_busio_spi_transfer(spi, out_data, in_data, len)) {
        mp_raise_OSError(MP_EIO);
    }

    return BUSIO_DMA_SPI_CHANNEL;
}

bool common_hal_busio_dma_spi_is_busy(uint dma_channel) {
    (void)dma_channel;
    return false;
}

uint common_hal_busio_dma_uart_read(busio_uart_obj_t *uart, uint8_t *data, size_t len) {
    int errcode = 0;
    size_t result = common_hal_busio_uart_read(uart, data, len, &errcode);
    if (result == MP_STREAM_ERROR) {
        mp_raise_OSError(errcode);
    }

    return BUSIO_DMA_UART_CHANNEL;
}

uint common_hal_busio_dma_uart_write(busio_uart_obj_t *uart, const uint8_t *data, size_t len) {
    int errcode = 0;
    size_t result = common_hal_busio_uart_write(uart, data, len, &errcode);
    if (result == MP_STREAM_ERROR) {
        mp_raise_OSError(errcode);
    }

    return BUSIO_DMA_UART_CHANNEL;
}

bool common_hal_busio_dma_uart_is_busy(uint dma_channel) {
    (void)dma_channel;
    return false;
}

#endif
