// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2026
//
// SPDX-License-Identifier: MIT

#include "shared-bindings/busio/dma.h"

#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/uart.h"

uint common_hal_busio_dma_i2c_read(busio_i2c_obj_t *i2c, uint8_t address, uint8_t *data, size_t len, bool nostop) {
    return i2c_read_dma(i2c->peripheral, address, data, len, nostop);
}

uint common_hal_busio_dma_i2c_write(busio_i2c_obj_t *i2c, uint8_t address, const uint8_t *data, size_t len, bool nostop) {
    return i2c_write_dma(i2c->peripheral, address, data, len, nostop);
}

bool common_hal_busio_dma_i2c_is_busy(uint dma_channel) {
    return i2c_dma_is_busy(dma_channel);
}

uint common_hal_busio_dma_spi_write(busio_spi_obj_t *spi, const uint8_t *data, size_t len) {
    return spi_write_dma(spi->peripheral, data, len);
}

uint common_hal_busio_dma_spi_read(busio_spi_obj_t *spi, uint8_t write_value, uint8_t *data, size_t len) {
    return spi_read_dma(spi->peripheral, write_value, data, len);
}

uint common_hal_busio_dma_spi_transfer(busio_spi_obj_t *spi, const uint8_t *out_data, uint8_t *in_data, size_t len) {
    return spi_write_read_dma(spi->peripheral, out_data, in_data, len);
}

bool common_hal_busio_dma_spi_is_busy(uint dma_channel) {
    return spi_dma_is_busy(dma_channel);
}

uint common_hal_busio_dma_uart_read(busio_uart_obj_t *uart, uint8_t *data, size_t len) {
    return uart_read_dma(uart->uart, data, len);
}

uint common_hal_busio_dma_uart_write(busio_uart_obj_t *uart, const uint8_t *data, size_t len) {
    return uart_write_dma(uart->uart, data, len);
}

bool common_hal_busio_dma_uart_is_busy(uint dma_channel) {
    return uart_dma_is_busy(dma_channel);
}
