// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2026
//
// SPDX-License-Identifier: MIT

#include "shared-bindings/busio/dma.h"

#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/uart.h"

typedef struct {
    bool write_pending_last;
    i2c_inst_t *i2c;
    uint8_t last_byte;
    bool nostop;
} i2c_dma_channel_state_t;

typedef struct {
    uint paired_rx_channel;
    uint8_t repeated_tx_data;
    uint8_t discard_rx_data;
} spi_dma_channel_state_t;

static i2c_dma_channel_state_t i2c_dma_channel_state[NUM_DMA_CHANNELS];
static spi_dma_channel_state_t spi_dma_channel_state[NUM_DMA_CHANNELS];

// Addresses of the form 000 0xxx or 111 1xxx are reserved. No slave should
// have these addresses.
#define i2c_reserved_addr(addr) (((addr) & 0x78) == 0 || ((addr) & 0x78) == 0x78)

static uint common_hal_i2c_write_dma(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop) {
    invalid_params_if(HARDWARE_I2C, addr >= 0x80); // 7-bit addresses
    invalid_params_if(HARDWARE_I2C, i2c_reserved_addr(addr));
    invalid_params_if(HARDWARE_I2C, len == 0);
    invalid_params_if(HARDWARE_I2C, ((int)len) < 0);

    uint dma_channel = dma_claim_unused_channel(true);
    if (len > i2c_get_write_available(i2c)) {
        dma_channel_unclaim(dma_channel);
        return PICO_ERROR_INSUFFICIENT_RESOURCES;
    }

    i2c->hw->enable = 0;
    i2c->hw->tar = addr;
    i2c->hw->enable = 1;

    i2c_dma_channel_state[dma_channel].write_pending_last = false;
    i2c_dma_channel_state[dma_channel].i2c = i2c;
    i2c_dma_channel_state[dma_channel].last_byte = 0;
    i2c_dma_channel_state[dma_channel].nostop = nostop;

    if (len == 1) {
        i2c->hw->data_cmd =
            bool_to_bit(i2c->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
            bool_to_bit(!nostop) << I2C_IC_DATA_CMD_STOP_LSB |
            src[0];
        i2c->restart_on_next = nostop;
        return dma_channel;
    }

    i2c->hw->data_cmd =
        bool_to_bit(i2c->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
        src[0];

    size_t middle_len = len - 2;
    if (middle_len > 0) {
        dma_channel_config config = dma_channel_get_default_config(dma_channel);
        channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
        channel_config_set_read_increment(&config, true);
        channel_config_set_write_increment(&config, false);
        channel_config_set_dreq(&config, I2C_DREQ_NUM(i2c, true));

        dma_channel_configure(dma_channel, &config, &i2c->hw->data_cmd, src + 1, middle_len, true);
    }

    i2c_dma_channel_state[dma_channel].write_pending_last = true;
    i2c_dma_channel_state[dma_channel].last_byte = src[len - 1];
    i2c->restart_on_next = nostop;
    return dma_channel;
}

static uint common_hal_i2c_read_dma(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop) {
    invalid_params_if(HARDWARE_I2C, addr >= 0x80); // 7-bit addresses
    invalid_params_if(HARDWARE_I2C, i2c_reserved_addr(addr));
    invalid_params_if(HARDWARE_I2C, len == 0);
    invalid_params_if(HARDWARE_I2C, ((int)len) < 0);
    uint dma_channel = dma_claim_unused_channel(true);

    if (len > i2c_get_write_available(i2c)) {
        return PICO_ERROR_INSUFFICIENT_RESOURCES;
    }

    i2c->hw->enable = 0;
    i2c->hw->tar = addr;
    i2c->hw->enable = 1;

    dma_channel_config config = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
    channel_config_set_read_increment(&config, false);
    channel_config_set_write_increment(&config, true);
    channel_config_set_dreq(&config, I2C_DREQ_NUM(i2c, false));

    dma_channel_configure(dma_channel, &config, dst, &i2c->hw->data_cmd, len, true);

    for (size_t byte_ctr = 0; byte_ctr < len; ++byte_ctr) {
        bool first = byte_ctr == 0;
        bool last = byte_ctr == len - 1;

        i2c->hw->data_cmd =
            bool_to_bit(first && i2c->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
            bool_to_bit(last && !nostop) << I2C_IC_DATA_CMD_STOP_LSB |
            I2C_IC_DATA_CMD_CMD_BITS; // -> 1 for read
    }

    i2c->restart_on_next = nostop;
    return dma_channel;
}

static bool common_hal_i2c_dma_is_busy(uint dma_channel) {
    if (dma_channel_is_busy(dma_channel)) {
        return true;
    }

    i2c_dma_channel_state_t *state = &i2c_dma_channel_state[dma_channel];
    if (state->write_pending_last) {
        if (!i2c_get_write_available(state->i2c)) {
            return true;
        }

        state->i2c->hw->data_cmd =
            bool_to_bit(!state->nostop) << I2C_IC_DATA_CMD_STOP_LSB |
            state->last_byte;
        state->write_pending_last = false;
        state->i2c = NULL;
        return true;
    }

    dma_channel_unclaim(dma_channel);
    return false;
}

static uint common_hal_spi_write_dma(spi_inst_t *spi, const uint8_t *src, size_t len) {
    invalid_params_if(HARDWARE_SPI, 0 > (int)len);
    invalid_params_if(HARDWARE_SPI, len == 0);

    uint tx_dma_channel = dma_claim_unused_channel(true);
    uint rx_dma_channel = dma_claim_unused_channel(true);

    spi_dma_channel_state[tx_dma_channel].paired_rx_channel = rx_dma_channel;
    spi_dma_channel_state[tx_dma_channel].discard_rx_data = 0;

    dma_channel_config tx_config = dma_channel_get_default_config(tx_dma_channel);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_config, true);
    channel_config_set_write_increment(&tx_config, false);
    channel_config_set_dreq(&tx_config, SPI_DREQ_NUM(spi, true));
    dma_channel_configure(tx_dma_channel, &tx_config, &spi_get_hw(spi)->dr, src, len, false);

    dma_channel_config rx_config = dma_channel_get_default_config(rx_dma_channel);
    channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_config, false);
    channel_config_set_write_increment(&rx_config, false);
    channel_config_set_dreq(&rx_config, SPI_DREQ_NUM(spi, false));
    dma_channel_configure(rx_dma_channel, &rx_config, &spi_dma_channel_state[tx_dma_channel].discard_rx_data,
        &spi_get_hw(spi)->dr, len, false);

    dma_start_channel_mask((1u << tx_dma_channel) | (1u << rx_dma_channel));
    return tx_dma_channel;
}

static uint common_hal_spi_read_dma(spi_inst_t *spi, uint8_t repeated_tx_data, uint8_t *dst, size_t len) {
    invalid_params_if(HARDWARE_SPI, 0 > (int)len);
    invalid_params_if(HARDWARE_SPI, len == 0);

    uint tx_dma_channel = dma_claim_unused_channel(true);
    uint rx_dma_channel = dma_claim_unused_channel(true);

    spi_dma_channel_state[tx_dma_channel].paired_rx_channel = rx_dma_channel;
    spi_dma_channel_state[tx_dma_channel].repeated_tx_data = repeated_tx_data;

    dma_channel_config tx_config = dma_channel_get_default_config(tx_dma_channel);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_config, false);
    channel_config_set_write_increment(&tx_config, false);
    channel_config_set_dreq(&tx_config, SPI_DREQ_NUM(spi, true));
    dma_channel_configure(tx_dma_channel, &tx_config, &spi_get_hw(spi)->dr,
        &spi_dma_channel_state[tx_dma_channel].repeated_tx_data, len, false);

    dma_channel_config rx_config = dma_channel_get_default_config(rx_dma_channel);
    channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_config, false);
    channel_config_set_write_increment(&rx_config, true);
    channel_config_set_dreq(&rx_config, SPI_DREQ_NUM(spi, false));
    dma_channel_configure(rx_dma_channel, &rx_config, dst, &spi_get_hw(spi)->dr, len, false);

    dma_start_channel_mask((1u << tx_dma_channel) | (1u << rx_dma_channel));
    return tx_dma_channel;
}

static uint common_hal_spi_write_read_dma(spi_inst_t *spi, const uint8_t *src, uint8_t *dst, size_t len) {
    invalid_params_if(HARDWARE_SPI, 0 > (int)len);
    invalid_params_if(HARDWARE_SPI, len == 0);

    uint tx_dma_channel = dma_claim_unused_channel(true);
    uint rx_dma_channel = dma_claim_unused_channel(true);

    spi_dma_channel_state[tx_dma_channel].paired_rx_channel = rx_dma_channel;

    dma_channel_config tx_config = dma_channel_get_default_config(tx_dma_channel);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_config, true);
    channel_config_set_write_increment(&tx_config, false);
    channel_config_set_dreq(&tx_config, SPI_DREQ_NUM(spi, true));
    dma_channel_configure(tx_dma_channel, &tx_config, &spi_get_hw(spi)->dr, src, len, false);

    dma_channel_config rx_config = dma_channel_get_default_config(rx_dma_channel);
    channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_config, false);
    channel_config_set_write_increment(&rx_config, true);
    channel_config_set_dreq(&rx_config, SPI_DREQ_NUM(spi, false));
    dma_channel_configure(rx_dma_channel, &rx_config, dst, &spi_get_hw(spi)->dr, len, false);

    dma_start_channel_mask((1u << tx_dma_channel) | (1u << rx_dma_channel));
    return tx_dma_channel;
}

static bool common_hal_spi_dma_is_busy(uint dma_channel) {
    if (dma_channel_is_busy(dma_channel)) {
        return true;
    }

    uint rx_dma_channel = spi_dma_channel_state[dma_channel].paired_rx_channel;
    if (rx_dma_channel < NUM_DMA_CHANNELS && dma_channel_is_busy(rx_dma_channel)) {
        return true;
    }

    if (rx_dma_channel < NUM_DMA_CHANNELS) {
        dma_channel_unclaim(rx_dma_channel);
    }
    dma_channel_unclaim(dma_channel);

    spi_dma_channel_state[dma_channel].paired_rx_channel = NUM_DMA_CHANNELS;
    return false;
}

static uint common_hal_uart_write_dma(uart_inst_t *uart, const uint8_t *src, size_t len) {
    invalid_params_if(HARDWARE_UART, 0 > (int)len);
    invalid_params_if(HARDWARE_UART, len == 0);

    uint dma_channel = dma_claim_unused_channel(true);

    dma_channel_config config = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
    channel_config_set_read_increment(&config, true);
    channel_config_set_write_increment(&config, false);
    channel_config_set_dreq(&config, UART_DREQ_NUM(uart, true));

    dma_channel_configure(dma_channel, &config, &uart_get_hw(uart)->dr, src, len, true);
    return dma_channel;
}

static uint common_hal_uart_read_dma(uart_inst_t *uart, uint8_t *dst, size_t len) {
    invalid_params_if(HARDWARE_UART, 0 > (int)len);
    invalid_params_if(HARDWARE_UART, len == 0);

    uint dma_channel = dma_claim_unused_channel(true);

    dma_channel_config config = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
    channel_config_set_read_increment(&config, false);
    channel_config_set_write_increment(&config, true);
    channel_config_set_dreq(&config, UART_DREQ_NUM(uart, false));

    dma_channel_configure(dma_channel, &config, dst, &uart_get_hw(uart)->dr, len, true);
    return dma_channel;
}

static bool common_hal_uart_dma_is_busy(uint dma_channel) {
    if (dma_channel_is_busy(dma_channel)) {
        return true;
    }
    dma_channel_unclaim(dma_channel);
    return false;
}

uint common_hal_busio_dma_i2c_read(busio_i2c_obj_t *i2c, uint8_t address, uint8_t *data, size_t len, bool nostop) {
    return common_hal_i2c_read_dma(i2c->peripheral, address, data, len, nostop);
}

uint common_hal_busio_dma_i2c_write(busio_i2c_obj_t *i2c, uint8_t address, const uint8_t *data, size_t len, bool nostop) {
    return common_hal_i2c_write_dma(i2c->peripheral, address, data, len, nostop);
}

bool common_hal_busio_dma_i2c_is_busy(uint dma_channel) {
    return common_hal_i2c_dma_is_busy(dma_channel);
}

uint common_hal_busio_dma_spi_write(busio_spi_obj_t *spi, const uint8_t *data, size_t len) {
    return common_hal_spi_write_dma(spi->peripheral, data, len);
}

uint common_hal_busio_dma_spi_read(busio_spi_obj_t *spi, uint8_t write_value, uint8_t *data, size_t len) {
    return common_hal_spi_read_dma(spi->peripheral, write_value, data, len);
}

uint common_hal_busio_dma_spi_transfer(busio_spi_obj_t *spi, const uint8_t *out_data, uint8_t *in_data, size_t len) {
    return common_hal_spi_write_read_dma(spi->peripheral, out_data, in_data, len);
}

bool common_hal_busio_dma_spi_is_busy(uint dma_channel) {
    return common_hal_spi_dma_is_busy(dma_channel);
}

uint common_hal_busio_dma_uart_read(busio_uart_obj_t *uart, uint8_t *data, size_t len) {
    return common_hal_uart_read_dma(uart->uart, data, len);
}

uint common_hal_busio_dma_uart_write(busio_uart_obj_t *uart, const uint8_t *data, size_t len) {
    return common_hal_uart_write_dma(uart->uart, data, len);
}

bool common_hal_busio_dma_uart_is_busy(uint dma_channel) {
    return common_hal_uart_dma_is_busy(dma_channel);
}
