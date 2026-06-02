// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2021 Carl Kugler III
// SPDX-FileCopyrightText: Copyright (c) 2025 CircuitPython Contributors
//
// SPDX-License-Identifier: Apache-2.0
//
// RP2350 SDIO transport layer — PIO + DMA implementation.
// Derived from carlk3/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico (Apache-2.0):
//   https://github.com/carlk3/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// PIO-program constants: CLKDIV, SDIO_CLK_PIN_D0_OFFSET, etc.
#include "rp2350_sdio.pio.h"

#include "hardware/pio.h"
#include "hardware/dma.h"

// Maximum number of blocks that can be transferred in one multi-block DMA burst.
#define RP2350_SDIO_MAX_BLOCKS  128
#define RP2350_SDIO_BLOCK_SIZE  512
#define RP2350_SDIO_WORDS_PER_BLOCK (RP2350_SDIO_BLOCK_SIZE / sizeof(uint32_t))

// Return codes from transport functions.
typedef enum {
    SDIO_OK = 0,
    SDIO_BUSY,
    SDIO_ERR_RESPONSE_TIMEOUT,
    SDIO_ERR_RESPONSE_CRC,
    SDIO_ERR_RESPONSE_CODE,
    SDIO_ERR_DATA_TIMEOUT,
    SDIO_ERR_DATA_CRC,
    SDIO_ERR_WRITE_CRC,
    SDIO_ERR_WRITE_FAIL,
    SDIO_ERR_INIT,
} sdio_status_t;

// Internal transfer state machine.
typedef enum {
    SDIO_TRANSFER_IDLE = 0,
    SDIO_TRANSFER_RX,
    SDIO_TRANSFER_TX,
    SDIO_TRANSFER_TX_WAIT_IDLE,
} sdio_transfer_state_t;

// One DMA control-block descriptor used by the chained RX DMA.
// Layout matches the two fields written by the control channel into
// dma_hw->ch[n].al1_write_addr + al1_transfer_count.
typedef struct {
    uint32_t *write_addr;
    uint32_t transfer_count;
} sdio_dma_ctrl_t;

// Two-word container for per-block received CRC (8 bytes of 4-line CRC).
typedef struct {
    uint32_t top;
    uint32_t bottom;
} sdio_rx_crc_t;

// All PIO/DMA state for one SDIO interface.
typedef struct {
    // PIO instance and state machines.
    PIO pio;
    uint cmd_sm;        // SM0: clock + CMD line
    uint data_sm;       // SM1: 4-bit data (shared between RX and TX programs)
    uint pio_cmd_clk_offset;
    uint pio_data_rx_offset;
    uint pio_data_tx_offset;
    pio_sm_config pio_cfg_data_rx;
    pio_sm_config pio_cfg_data_tx;

    // GPIO pin numbers.
    uint clk_gpio;   // CLK = (d0_gpio - 2 + 32) % 32
    uint cmd_gpio;
    uint d0_gpio;    // D1-D3 are implicit: d0+1, d0+2, d0+3

    // Claimed DMA channels.
    int dma_ch;      // Primary channel (data payload)
    int dma_chb;     // Secondary channel (control / CRC / chained)

    // Multi-block transfer bookkeeping.
    sdio_transfer_state_t transfer_state;
    uint32_t *data_buf;
    uint32_t blocks_done;
    uint32_t total_blocks;
    uint32_t blocks_checksumed;
    uint32_t checksum_errors;
    uint64_t transfer_start_ms;
    sdio_status_t wr_status;
    uint32_t card_response;         // write response token from card
    uint64_t next_wr_block_checksum;
    uint32_t end_token_buf[3];      // [crc_high, crc_low, 0xFFFFFFFF]

    // Chained-DMA control blocks for multi-block RX.
    // Index: [i*2] = data block i, [i*2+1] = CRC for block i.
    // Final entry is sentinel (zero count) that stops the chain.
    sdio_dma_ctrl_t dma_blocks[RP2350_SDIO_MAX_BLOCKS * 2 + 1];

    // Received CRC words for each block (verified after transfer).
    sdio_rx_crc_t received_checksums[RP2350_SDIO_MAX_BLOCKS];

    bool resources_claimed;
    float clk_div;          // PIO clock divider (system_clock / (4 * sdio_hz))
} rp2350_sdio_state_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

// Initialise PIO state machines and claim DMA channels.
// clk_div: PIO clock divider = sys_clk / (CLKDIV * desired_hz).
bool rp2350_sdio_init(rp2350_sdio_state_t *state, uint clk_gpio, uint cmd_gpio,
    uint d0_gpio, float clk_div);

// Release PIO/DMA resources.
void rp2350_sdio_deinit(rp2350_sdio_state_t *state);

// Update clock divider (e.g. after card init, switch to full speed).
void rp2350_sdio_set_clkdiv(rp2350_sdio_state_t *state, float clk_div);

// Send a command and receive an R1 / R3 / R6 / R7 response (48-bit).
// Pass response=NULL to skip reading the response.
sdio_status_t rp2350_sdio_command_R1(rp2350_sdio_state_t *state,
    uint8_t command, uint32_t arg, uint32_t *response);

// Variant that ignores CRC in the response (used for ACMD41 / R3).
sdio_status_t rp2350_sdio_command_R3(rp2350_sdio_state_t *state,
    uint8_t command, uint32_t arg, uint32_t *response);

// Send a command and receive an R2 response (136-bit CID/CSD).
// response must point to a 16-byte buffer.
sdio_status_t rp2350_sdio_command_R2(rp2350_sdio_state_t *state,
    uint8_t command, uint32_t arg, uint8_t *response);

// Start a multi-block DMA read.  Buffer must be 4-byte aligned.
sdio_status_t rp2350_sdio_rx_start(rp2350_sdio_state_t *state,
    uint8_t *buf, uint32_t num_blocks);

// Poll for RX completion.  Returns SDIO_OK, SDIO_BUSY, or an error.
sdio_status_t rp2350_sdio_rx_poll(rp2350_sdio_state_t *state);

// Start a multi-block DMA write.  Buffer must be 4-byte aligned.
sdio_status_t rp2350_sdio_tx_start(rp2350_sdio_state_t *state,
    const uint8_t *buf, uint32_t num_blocks);

// Poll for TX completion.  Returns SDIO_OK, SDIO_BUSY, or an error.
sdio_status_t rp2350_sdio_tx_poll(rp2350_sdio_state_t *state);

// Abort any in-progress transfer and return bus to idle.
sdio_status_t rp2350_sdio_stop(rp2350_sdio_state_t *state);
