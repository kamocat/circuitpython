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
// Modifications: adapted for RP2350, restructured as a stateless module
// with CircuitPython pin/DMA allocation conventions, removed FatFS and
// host-specific dependencies, renamed symbols to rp2350_sdio_* prefix.

#include <string.h>
#include <assert.h>

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "py/runtime.h"

#include "shared-bindings/time/__init__.h"

#include "rp2350_sdio.h"
#include "rp2350_sdio.pio.h"

// ---------------------------------------------------------------------------
// Timing helpers
// ---------------------------------------------------------------------------

static inline uint64_t sdio_millis(void) {
    return common_hal_time_monotonic_ms();
}

// Per SD spec, NCR (command response time) is 64 CLK cycles max at 400 kHz
// init speed = 0.16 ms.  Use 1 ms to give generous margin for slow cards;
// this is the per-attempt ceiling, not the total init time.
// Note: ACMD41 polls rely on this — a lower value means tighter retry loops
// during card power-up (the outer deadline in SDCard.c is the real 2s limit).
#define SDIO_TIMEOUT_CMD_MS   1u
#define SDIO_TIMEOUT_DATA_MS  100u
#define SDIO_TIMEOUT_WRITE_MS 100u

// ---------------------------------------------------------------------------
// CRC-7 lookup table (used in command packets).
// ---------------------------------------------------------------------------

static const uint8_t crc7_table[256] = {
    0x00, 0x12, 0x24, 0x36, 0x48, 0x5a, 0x6c, 0x7e, 0x90, 0x82, 0xb4, 0xa6, 0xd8, 0xca, 0xfc, 0xee,
    0x32, 0x20, 0x16, 0x04, 0x7a, 0x68, 0x5e, 0x4c, 0xa2, 0xb0, 0x86, 0x94, 0xea, 0xf8, 0xce, 0xdc,
    0x64, 0x76, 0x40, 0x52, 0x2c, 0x3e, 0x08, 0x1a, 0xf4, 0xe6, 0xd0, 0xc2, 0xbc, 0xae, 0x98, 0x8a,
    0x56, 0x44, 0x72, 0x60, 0x1e, 0x0c, 0x3a, 0x28, 0xc6, 0xd4, 0xe2, 0xf0, 0x8e, 0x9c, 0xaa, 0xb8,
    0xc8, 0xda, 0xec, 0xfe, 0x80, 0x92, 0xa4, 0xb6, 0x58, 0x4a, 0x7c, 0x6e, 0x10, 0x02, 0x34, 0x26,
    0xfa, 0xe8, 0xde, 0xcc, 0xb2, 0xa0, 0x96, 0x84, 0x6a, 0x78, 0x4e, 0x5c, 0x22, 0x30, 0x06, 0x14,
    0xac, 0xbe, 0x88, 0x9a, 0xe4, 0xf6, 0xc0, 0xd2, 0x3c, 0x2e, 0x18, 0x0a, 0x74, 0x66, 0x50, 0x42,
    0x9e, 0x8c, 0xba, 0xa8, 0xd6, 0xc4, 0xf2, 0xe0, 0x0e, 0x1c, 0x2a, 0x38, 0x46, 0x54, 0x62, 0x70,
    0x82, 0x90, 0xa6, 0xb4, 0xca, 0xd8, 0xee, 0xfc, 0x12, 0x00, 0x36, 0x24, 0x5a, 0x48, 0x7e, 0x6c,
    0xb0, 0xa2, 0x94, 0x86, 0xf8, 0xea, 0xdc, 0xce, 0x20, 0x32, 0x04, 0x16, 0x68, 0x7a, 0x4c, 0x5e,
    0xe6, 0xf4, 0xc2, 0xd0, 0xae, 0xbc, 0x8a, 0x98, 0x76, 0x64, 0x52, 0x40, 0x3e, 0x2c, 0x1a, 0x08,
    0xd4, 0xc6, 0xf0, 0xe2, 0x9c, 0x8e, 0xb8, 0xaa, 0x44, 0x56, 0x60, 0x72, 0x0c, 0x1e, 0x28, 0x3a,
    0x4a, 0x58, 0x6e, 0x7c, 0x02, 0x10, 0x26, 0x34, 0xda, 0xc8, 0xfe, 0xec, 0x92, 0x80, 0xb6, 0xa4,
    0x78, 0x6a, 0x5c, 0x4e, 0x30, 0x22, 0x14, 0x06, 0xe8, 0xfa, 0xcc, 0xde, 0xa0, 0xb2, 0x84, 0x96,
    0x2e, 0x3c, 0x0a, 0x18, 0x66, 0x74, 0x42, 0x50, 0xbe, 0xac, 0x9a, 0x88, 0xf6, 0xe4, 0xd2, 0xc0,
    0x1c, 0x0e, 0x38, 0x2a, 0x54, 0x46, 0x70, 0x62, 0x8c, 0x9e, 0xa8, 0xba, 0xc4, 0xd6, 0xe0, 0xf2
};

// ---------------------------------------------------------------------------
// CRC-16 over 4 parallel data lines.
// ---------------------------------------------------------------------------

__attribute__((optimize("Ofast")))
static uint64_t sdio_crc16_4bit(const uint32_t *data, uint32_t num_words) {
    uint64_t crc = 0;
    const uint32_t *end = data + num_words;
    while (data < end) {
        for (int i = 0; i < 4; i++) {
            uint32_t data_in = __builtin_bswap32(*data++);
            uint32_t data_out = (uint32_t)(crc >> 32);
            crc <<= 32;
            data_out ^= data_out >> 16;
            data_out ^= data_in >> 16;
            uint64_t xorred = data_out ^ data_in;
            crc ^= xorred;
            crc ^= xorred << (5 * 4);
            crc ^= xorred << (12 * 4);
        }
    }
    return crc;
}

// ---------------------------------------------------------------------------
// Send a command frame to the CMD SM TX FIFO.
// ---------------------------------------------------------------------------

static void sdio_send_command(rp2350_sdio_state_t *s,
    uint8_t command, uint32_t arg, uint8_t response_bits) {
    // Build two 32-bit words that match the PIO TX FIFO format:
    //   word0[31:24] = 47 (total cmd bits - 1)
    //   word0[23:22] = 0b01 (start + direction bit = 1 for host→card)
    //   word0[21:16] = command index
    //   word0[15:08] = arg[31:24]
    //   word0[07:00] = arg[23:16]
    //   word1[31:24] = arg[15:8]
    //   word1[23:16] = arg[7:0]
    //   word1[15:08] = CRC7 << 1 | 1 (end bit)
    //   word1[07:00] = response_bits - 1 (or 0 if no response)
    uint32_t word0 =
        (47u << 24) |
        (1u << 22) |
        ((uint32_t)command << 16) |
        (((arg >> 24) & 0xFF) << 8) |
        (((arg >> 16) & 0xFF) << 0);

    uint32_t word1 =
        (((arg >> 8) & 0xFF) << 24) |
        (((arg >> 0) & 0xFF) << 16) |
        (1u << 8); // end bit

    if (response_bits) {
        word1 |= (uint32_t)(response_bits - 1);
    }

    // CRC over the bytes that will be sent (big-endian order).
    uint8_t crc = 0;
    crc = crc7_table[crc ^ ((word0 >> 16) & 0xFF)];
    crc = crc7_table[crc ^ ((word0 >> 8) & 0xFF)];
    crc = crc7_table[crc ^ ((word0 >> 0) & 0xFF)];
    crc = crc7_table[crc ^ ((word1 >> 24) & 0xFF)];
    crc = crc7_table[crc ^ ((word1 >> 16) & 0xFF)];
    // Insert CRC into word1[15:8]: CRC value in bits [7:1], end-bit in bit 0.
    word1 = (word1 & 0xFFFF00FFu) | ((uint32_t)(crc | 1) << 8);

    // The PIO program has no .wrap directive, so it uses the default: wrap from
    // the last instruction (resp_done: push) back to instruction 0 (mov OSR, NULL).
    // OSR is therefore always pre-filled with zeros before wait_cmd — after every
    // successful command via the program wrap, and after timeout recovery via the
    // explicit `jmp 0`.  No dummy word is ever needed.
    pio_sm_clear_fifos(s->pio, s->cmd_sm);
    pio_sm_put(s->pio, s->cmd_sm, word0);
    pio_sm_put(s->pio, s->cmd_sm, word1);
}

// ---------------------------------------------------------------------------
// Command helpers
// ---------------------------------------------------------------------------

sdio_status_t rp2350_sdio_command_R1(rp2350_sdio_state_t *s,
    uint8_t command, uint32_t arg, uint32_t *response) {
    sdio_send_command(s, command, arg, response ? 48 : 0);

    uint32_t wait_words = response ? 2u : 1u;
    uint64_t deadline = sdio_millis() + SDIO_TIMEOUT_CMD_MS;
    while (pio_sm_get_rx_fifo_level(s->pio, s->cmd_sm) < wait_words) {
        if (sdio_millis() > deadline) {
            pio_sm_clear_fifos(s->pio, s->cmd_sm);
            pio_sm_exec(s->pio, s->cmd_sm,
                pio_encode_jmp(s->pio_cmd_clk_offset));
            return SDIO_ERR_RESPONSE_TIMEOUT;
        }
    }

    if (response) {
        uint32_t resp0 = pio_sm_get(s->pio, s->cmd_sm);
        uint32_t resp1 = pio_sm_get(s->pio, s->cmd_sm);

        uint8_t crc = 0;
        crc = crc7_table[crc ^ ((resp0 >> 24) & 0xFF)];
        crc = crc7_table[crc ^ ((resp0 >> 16) & 0xFF)];
        crc = crc7_table[crc ^ ((resp0 >> 8) & 0xFF)];
        crc = crc7_table[crc ^ ((resp0 >> 0) & 0xFF)];
        crc = crc7_table[crc ^ ((resp1 >> 8) & 0xFF)];
        uint8_t actual_crc = (uint8_t)(resp1 >> 0) & 0xFE;
        if (crc != actual_crc) {
            return SDIO_ERR_RESPONSE_CRC;
        }

        uint8_t resp_cmd = (uint8_t)(resp0 >> 24);
        // ACMD41 is a special case: response command code is 0x3F (R3 format).
        if (resp_cmd != command && command != 41) {
            return SDIO_ERR_RESPONSE_CODE;
        }

        *response = (uint32_t)(((resp0 & 0xFFFFFF) << 8) | ((resp1 >> 8) & 0xFF));
    } else {
        pio_sm_get(s->pio, s->cmd_sm); // consume dummy marker
    }
    return SDIO_OK;
}

sdio_status_t rp2350_sdio_command_R3(rp2350_sdio_state_t *s,
    uint8_t command, uint32_t arg, uint32_t *response) {
    // R3 has no CRC — send with 48 response bits, skip CRC check.
    sdio_send_command(s, command, arg, 48);

    uint64_t deadline = sdio_millis() + SDIO_TIMEOUT_CMD_MS;
    while (pio_sm_get_rx_fifo_level(s->pio, s->cmd_sm) < 2) {
        if (sdio_millis() > deadline) {
            pio_sm_clear_fifos(s->pio, s->cmd_sm);
            pio_sm_exec(s->pio, s->cmd_sm,
                pio_encode_jmp(s->pio_cmd_clk_offset));
            return SDIO_ERR_RESPONSE_TIMEOUT;
        }
    }
    uint32_t resp0 = pio_sm_get(s->pio, s->cmd_sm);
    uint32_t resp1 = pio_sm_get(s->pio, s->cmd_sm);
    if (response) {
        *response = (uint32_t)(((resp0 & 0xFFFFFF) << 8) | ((resp1 >> 8) & 0xFF));
    }
    return SDIO_OK;
}

sdio_status_t rp2350_sdio_command_R2(rp2350_sdio_state_t *s,
    uint8_t command, uint32_t arg, uint8_t *response) {
    // R2 is 136 bits.  Use DMA to receive 5 words (160 bits, PIO pads to 32-bit boundaries).
    pio_sm_clear_fifos(s->pio, s->cmd_sm);

    uint32_t response_buf[5] = {0};
    dma_channel_config cfg = dma_channel_get_default_config((uint)s->dma_ch);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, pio_get_dreq(s->pio, s->cmd_sm, false));
    dma_channel_configure((uint)s->dma_ch, &cfg,
        response_buf, &s->pio->rxf[s->cmd_sm], 5, true);

    sdio_send_command(s, command, arg, 136);

    uint64_t deadline = sdio_millis() + SDIO_TIMEOUT_CMD_MS;
    while (dma_channel_is_busy((uint)s->dma_ch)) {
        if (sdio_millis() > deadline) {
            dma_channel_abort((uint)s->dma_ch);
            pio_sm_clear_fifos(s->pio, s->cmd_sm);
            pio_sm_exec(s->pio, s->cmd_sm,
                pio_encode_jmp(s->pio_cmd_clk_offset));
            return SDIO_ERR_RESPONSE_TIMEOUT;
        }
    }
    dma_channel_abort((uint)s->dma_ch);

    // Unpack 16 response bytes from the 5-word buffer.
    response[ 0] = (response_buf[0] >> 16) & 0xFF;
    response[ 1] = (response_buf[0] >> 8) & 0xFF;
    response[ 2] = (response_buf[0] >> 0) & 0xFF;
    response[ 3] = (response_buf[1] >> 24) & 0xFF;
    response[ 4] = (response_buf[1] >> 16) & 0xFF;
    response[ 5] = (response_buf[1] >> 8) & 0xFF;
    response[ 6] = (response_buf[1] >> 0) & 0xFF;
    response[ 7] = (response_buf[2] >> 24) & 0xFF;
    response[ 8] = (response_buf[2] >> 16) & 0xFF;
    response[ 9] = (response_buf[2] >> 8) & 0xFF;
    response[10] = (response_buf[2] >> 0) & 0xFF;
    response[11] = (response_buf[3] >> 24) & 0xFF;
    response[12] = (response_buf[3] >> 16) & 0xFF;
    response[13] = (response_buf[3] >> 8) & 0xFF;
    response[14] = (response_buf[3] >> 0) & 0xFF;
    response[15] = (response_buf[4] >> 0) & 0xFF;

    // Validate CRC.
    uint8_t crc = 0;
    for (int i = 0; i < 15; i++) {
        crc = crc7_table[crc ^ response[i]];
    }
    uint8_t actual_crc = response[15] & 0xFE;
    if (crc != actual_crc) {
        return SDIO_ERR_RESPONSE_CRC;
    }
    // R2 always returns command code 0x3F.
    if ((response_buf[0] >> 24) != 0x3F) {
        return SDIO_ERR_RESPONSE_CODE;
    }
    return SDIO_OK;
}

// ---------------------------------------------------------------------------
// Multi-block DMA read
// ---------------------------------------------------------------------------

sdio_status_t rp2350_sdio_rx_start(rp2350_sdio_state_t *s,
    uint8_t *buf, uint32_t num_blocks) {
    assert(((uintptr_t)buf & 3) == 0);
    assert(num_blocks > 0 && num_blocks <= RP2350_SDIO_MAX_BLOCKS);

    s->transfer_state = SDIO_TRANSFER_RX;
    s->transfer_start_ms = sdio_millis();
    s->data_buf = (uint32_t *)buf;
    s->blocks_done = 0;
    s->total_blocks = num_blocks;
    s->blocks_checksumed = 0;
    s->checksum_errors = 0;

    // Build DMA control-block descriptors:
    //   entry [i*2]   → 512-byte data payload
    //   entry [i*2+1] → 8-byte CRC (2 words)
    for (uint32_t i = 0; i < num_blocks; i++) {
        s->dma_blocks[i * 2].write_addr = (uint32_t *)(buf + i * RP2350_SDIO_BLOCK_SIZE);
        s->dma_blocks[i * 2].transfer_count = RP2350_SDIO_WORDS_PER_BLOCK;
        s->dma_blocks[i * 2 + 1].write_addr = &s->received_checksums[i].top;
        s->dma_blocks[i * 2 + 1].transfer_count = 2;
    }
    // Sentinel — zero count stops the chain.
    s->dma_blocks[num_blocks * 2].write_addr = NULL;
    s->dma_blocks[num_blocks * 2].transfer_count = 0;

    // Primary DMA channel: PIO RX FIFO → user buffer.
    // Byte-swap to restore network byte order.
    dma_channel_config cfg = dma_channel_get_default_config((uint)s->dma_ch);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, pio_get_dreq(s->pio, s->data_sm, false));
    channel_config_set_bswap(&cfg, true);
    channel_config_set_chain_to(&cfg, (uint)s->dma_chb);
    dma_channel_configure((uint)s->dma_ch, &cfg,
        NULL, &s->pio->rxf[s->data_sm], 0, false);

    // Secondary DMA channel: reconfigures the primary channel.
    // Reads pairs of words (write_addr + transfer_count) from dma_blocks[].
    // ring=3 means wrap at 8-byte boundary (one descriptor = 2 × uint32).
    dma_channel_config cfgb = dma_channel_get_default_config((uint)s->dma_chb);
    channel_config_set_transfer_data_size(&cfgb, DMA_SIZE_32);
    channel_config_set_read_increment(&cfgb, true);
    channel_config_set_write_increment(&cfgb, true);
    channel_config_set_ring(&cfgb, true, 3); // wrap write at 8 bytes (al1 pair)
    dma_channel_configure((uint)s->dma_chb, &cfgb,
        &dma_hw->ch[s->dma_ch].al1_write_addr,
        s->dma_blocks, 2, false);

    // Configure data SM for reception.
    pio_sm_init(s->pio, s->data_sm, s->pio_data_rx_offset, &s->pio_cfg_data_rx);
    pio_sm_set_consecutive_pindirs(s->pio, s->data_sm, s->d0_gpio, 4, false);

    // Load nibble count into Y: block_size_bytes * 2 nibbles + 16 CRC nibbles - 1.
    uint32_t nibble_count = RP2350_SDIO_BLOCK_SIZE * 2 + 16 - 1;
    pio_sm_put(s->pio, s->data_sm, nibble_count);
    pio_sm_exec(s->pio, s->data_sm, pio_encode_out(pio_y, 32));

    // Join RX FIFO for deeper buffering during DMA block switching.
    s->pio->sm[s->data_sm].shiftctrl |= PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS;

    // Start control DMA first, then enable PIO SM.
    dma_channel_start((uint)s->dma_chb);
    pio_sm_set_enabled(s->pio, s->data_sm, true);
    return SDIO_OK;
}

static void sdio_verify_rx_checksums(rp2350_sdio_state_t *s, uint32_t maxcount) {
    while (s->blocks_checksumed < s->blocks_done && maxcount-- > 0) {
        uint32_t idx = s->blocks_checksumed++;
        uint64_t calculated = sdio_crc16_4bit(
            s->data_buf + idx * RP2350_SDIO_WORDS_PER_BLOCK,
            RP2350_SDIO_WORDS_PER_BLOCK);
        uint32_t top = __builtin_bswap32(s->received_checksums[idx].top);
        uint32_t bottom = __builtin_bswap32(s->received_checksums[idx].bottom);
        uint64_t expected = ((uint64_t)top << 32) | bottom;
        if (calculated != expected) {
            s->checksum_errors++;
        }
    }
}

sdio_status_t rp2350_sdio_rx_poll(rp2350_sdio_state_t *s) {
    // Use the control-channel read pointer to compute how many blocks are done.
    uint32_t ctrl_count =
        (dma_hw->ch[s->dma_chb].read_addr - (uintptr_t)s->dma_blocks)
        / sizeof(s->dma_blocks[0]);
    s->blocks_done = (ctrl_count > 0) ? (ctrl_count - 1) / 2 : 0;

    sdio_verify_rx_checksums(s, 4);

    if (s->blocks_done >= s->total_blocks) {
        s->transfer_state = SDIO_TRANSFER_IDLE;
        pio_sm_set_enabled(s->pio, s->data_sm, false);
        sdio_verify_rx_checksums(s, s->total_blocks);
        return (s->checksum_errors == 0) ? SDIO_OK : SDIO_ERR_DATA_CRC;
    }

    if (sdio_millis() - s->transfer_start_ms >= SDIO_TIMEOUT_DATA_MS) {
        rp2350_sdio_stop(s);
        return SDIO_ERR_DATA_TIMEOUT;
    }
    return SDIO_BUSY;
}

// ---------------------------------------------------------------------------
// Multi-block DMA write
// ---------------------------------------------------------------------------

static void sdio_start_block_tx(rp2350_sdio_state_t *s) {
    pio_sm_init(s->pio, s->data_sm, s->pio_data_tx_offset, &s->pio_cfg_data_tx);

    // Primary DMA: send 512 bytes of payload.
    dma_channel_config cfg = dma_channel_get_default_config((uint)s->dma_ch);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, pio_get_dreq(s->pio, s->data_sm, true));
    channel_config_set_bswap(&cfg, true);
    channel_config_set_chain_to(&cfg, (uint)s->dma_chb);
    dma_channel_configure((uint)s->dma_ch, &cfg,
        &s->pio->txf[s->data_sm],
        s->data_buf + s->blocks_done * RP2350_SDIO_WORDS_PER_BLOCK,
        RP2350_SDIO_WORDS_PER_BLOCK, false);

    // Build end-token buffer: [CRC_high, CRC_low, 0xFFFFFFFF].
    uint64_t crc = s->next_wr_block_checksum;
    s->end_token_buf[0] = (uint32_t)(crc >> 32);
    s->end_token_buf[1] = (uint32_t)(crc >> 0);
    s->end_token_buf[2] = 0xFFFFFFFFu;

    // Secondary DMA: send CRC + end token.
    dma_channel_config cfgb = dma_channel_get_default_config((uint)s->dma_chb);
    channel_config_set_transfer_data_size(&cfgb, DMA_SIZE_32);
    channel_config_set_read_increment(&cfgb, true);
    channel_config_set_write_increment(&cfgb, false);
    channel_config_set_dreq(&cfgb, pio_get_dreq(s->pio, s->data_sm, true));
    channel_config_set_bswap(&cfgb, false);
    dma_channel_configure((uint)s->dma_chb, &cfgb,
        &s->pio->txf[s->data_sm],
        s->end_token_buf, 3, false);

    // Load X (nibble count) and Y (response bit count) into SM.
    pio_sm_put(s->pio, s->data_sm, 1048); // 512*2 + 24 CRC nibbles
    pio_sm_exec(s->pio, s->data_sm, pio_encode_out(pio_x, 32));
    pio_sm_put(s->pio, s->data_sm, 31);   // 32 response bits
    pio_sm_exec(s->pio, s->data_sm, pio_encode_out(pio_y, 32));

    // Set D0-D3 high and as outputs before starting.
    pio_sm_exec(s->pio, s->data_sm, pio_encode_set(pio_pins, 15));
    pio_sm_exec(s->pio, s->data_sm, pio_encode_set(pio_pindirs, 15));

    // Write start token (0xFFFFFFF0 = start-bit followed by zeros on all 4 lines).
    pio_sm_put(s->pio, s->data_sm, 0xFFFFFFF0u);

    dma_channel_start((uint)s->dma_ch);
    pio_sm_set_enabled(s->pio, s->data_sm, true);
}

static void sdio_compute_tx_checksum(rp2350_sdio_state_t *s) {
    assert(s->blocks_checksumed < s->total_blocks);
    uint32_t idx = s->blocks_checksumed++;
    s->next_wr_block_checksum = sdio_crc16_4bit(
        s->data_buf + idx * RP2350_SDIO_WORDS_PER_BLOCK,
        RP2350_SDIO_WORDS_PER_BLOCK);
}

sdio_status_t rp2350_sdio_tx_start(rp2350_sdio_state_t *s,
    const uint8_t *buf, uint32_t num_blocks) {
    assert(((uintptr_t)buf & 3) == 0);
    assert(num_blocks > 0 && num_blocks <= RP2350_SDIO_MAX_BLOCKS);

    s->transfer_state = SDIO_TRANSFER_TX;
    s->transfer_start_ms = sdio_millis();
    s->data_buf = (uint32_t *)buf;
    s->blocks_done = 0;
    s->total_blocks = num_blocks;
    s->blocks_checksumed = 0;
    s->checksum_errors = 0;
    s->wr_status = SDIO_OK;

    sdio_compute_tx_checksum(s);
    sdio_start_block_tx(s);
    if (s->blocks_checksumed < s->total_blocks) {
        sdio_compute_tx_checksum(s);
    }
    return SDIO_OK;
}

static sdio_status_t check_write_response(uint32_t card_response) {
    uint32_t resp = card_response;
    if (!(~resp & 0xFFFF0000u)) {
        resp <<= 16;
    }
    if (!(~resp & 0xFF000000u)) {
        resp <<= 8;
    }
    if (!(~resp & 0xF0000000u)) {
        resp <<= 4;
    }
    if (!(~resp & 0xC0000000u)) {
        resp <<= 2;
    }
    if (!(~resp & 0x80000000u)) {
        resp <<= 1;
    }
    uint32_t wr_status = (resp >> 28) & 7;
    if (wr_status == 2) {
        return SDIO_OK;
    }
    if (wr_status == 5) {
        return SDIO_ERR_WRITE_CRC;
    }
    return SDIO_ERR_WRITE_FAIL;
}

// Drive the TX state machine forward.  Called in poll loop.
static void sdio_tx_fsm(rp2350_sdio_state_t *s) {
    if (s->transfer_state == SDIO_TRANSFER_TX) {
        if (!dma_channel_is_busy((uint)s->dma_ch) &&
            !dma_channel_is_busy((uint)s->dma_chb)) {
            s->transfer_state = SDIO_TRANSFER_TX_WAIT_IDLE;
            if (!pio_sm_is_rx_fifo_empty(s->pio, s->data_sm)) {
                s->card_response = pio_sm_get(s->pio, s->data_sm);
            } else {
                // Set up DMA to wait for the card response in the RX FIFO.
                dma_channel_config cfgb = dma_channel_get_default_config((uint)s->dma_chb);
                channel_config_set_transfer_data_size(&cfgb, DMA_SIZE_32);
                channel_config_set_read_increment(&cfgb, false);
                channel_config_set_write_increment(&cfgb, false);
                channel_config_set_dreq(&cfgb, pio_get_dreq(s->pio, s->data_sm, false));
                dma_channel_configure((uint)s->dma_chb, &cfgb,
                    &s->card_response, &s->pio->rxf[s->data_sm], 1, true);
            }
        }
    }

    if (s->transfer_state == SDIO_TRANSFER_TX_WAIT_IDLE) {
        if (!dma_channel_is_busy((uint)s->dma_chb)) {
            s->wr_status = check_write_response(s->card_response);
            if (s->wr_status != SDIO_OK) {
                rp2350_sdio_stop(s);
                return;
            }
            s->blocks_done++;
            if (s->blocks_done < s->total_blocks) {
                sdio_start_block_tx(s);
                s->transfer_state = SDIO_TRANSFER_TX;
                if (s->blocks_checksumed < s->total_blocks) {
                    sdio_compute_tx_checksum(s);
                }
            } else {
                rp2350_sdio_stop(s);
            }
        }
    }
}

sdio_status_t rp2350_sdio_tx_poll(rp2350_sdio_state_t *s) {
    sdio_tx_fsm(s);

    if (s->transfer_state == SDIO_TRANSFER_IDLE) {
        return s->wr_status;
    }
    if (sdio_millis() - s->transfer_start_ms >= SDIO_TIMEOUT_WRITE_MS) {
        rp2350_sdio_stop(s);
        return SDIO_ERR_DATA_TIMEOUT;
    }
    return SDIO_BUSY;
}

// ---------------------------------------------------------------------------
// Stop / abort
// ---------------------------------------------------------------------------

sdio_status_t rp2350_sdio_stop(rp2350_sdio_state_t *s) {
    dma_channel_abort((uint)s->dma_ch);
    dma_channel_abort((uint)s->dma_chb);
    pio_sm_set_enabled(s->pio, s->data_sm, false);
    pio_sm_set_consecutive_pindirs(s->pio, s->data_sm, s->d0_gpio, 4, false);
    s->transfer_state = SDIO_TRANSFER_IDLE;
    return SDIO_OK;
}

// ---------------------------------------------------------------------------
// Clock divider update
// ---------------------------------------------------------------------------

void rp2350_sdio_set_clkdiv(rp2350_sdio_state_t *s, float clk_div) {
    s->clk_div = clk_div;
    pio_sm_set_clkdiv(s->pio, s->cmd_sm, clk_div);
    // data SM picks up the new divider on the next pio_sm_init() call.
}

// ---------------------------------------------------------------------------
// Initialise
// ---------------------------------------------------------------------------

bool rp2350_sdio_init(rp2350_sdio_state_t *s, uint clk_gpio, uint cmd_gpio,
    uint d0_gpio, float clk_div) {
    if (!s->resources_claimed) {
        if (!s->pio) {
            s->pio = pio0;
        }
        s->cmd_sm = (uint)pio_claim_unused_sm(s->pio, true);
        s->data_sm = (uint)pio_claim_unused_sm(s->pio, true);
        s->dma_ch = dma_claim_unused_channel(true);
        s->dma_chb = dma_claim_unused_channel(true);
        s->resources_claimed = true;
    }

    s->clk_gpio = clk_gpio;
    s->cmd_gpio = cmd_gpio;
    s->d0_gpio = d0_gpio;
    s->clk_div = clk_div;

    dma_channel_abort((uint)s->dma_ch);
    dma_channel_abort((uint)s->dma_chb);
    pio_sm_set_enabled(s->pio, s->cmd_sm,  false);
    pio_sm_set_enabled(s->pio, s->data_sm, false);

    // Load PIO programs.  Clear instruction memory so we can share the PIO.
    pio_clear_instruction_memory(s->pio);

    // CMD+CLK state machine.
    s->pio_cmd_clk_offset = pio_add_program(s->pio, &sdio_cmd_clk_program);
    pio_sm_config cfg = sdio_cmd_clk_program_get_default_config(s->pio_cmd_clk_offset);
    sm_config_set_out_pins(&cfg, cmd_gpio, 1);
    sm_config_set_in_pins(&cfg, cmd_gpio);
    sm_config_set_set_pins(&cfg, cmd_gpio, 1);
    sm_config_set_jmp_pin(&cfg, cmd_gpio);
    sm_config_set_sideset_pins(&cfg, clk_gpio);
    sm_config_set_out_shift(&cfg, false, true, 32);
    sm_config_set_in_shift(&cfg, false, true, 32);
    sm_config_set_clkdiv(&cfg, clk_div);
    sm_config_set_mov_status(&cfg, STATUS_TX_LESSTHAN, 2);
    pio_sm_init(s->pio, s->cmd_sm, s->pio_cmd_clk_offset, &cfg);
    pio_sm_set_consecutive_pindirs(s->pio, s->cmd_sm, clk_gpio, 1, true);
    pio_sm_set_enabled(s->pio, s->cmd_sm, true);

    // Data RX program (loaded; SM only enabled during transfers).
    s->pio_data_rx_offset = pio_add_program(s->pio, &sdio_data_rx_program);
    s->pio_cfg_data_rx = sdio_data_rx_program_get_default_config(s->pio_data_rx_offset);
    sm_config_set_in_pins(&s->pio_cfg_data_rx, d0_gpio);
    sm_config_set_in_shift(&s->pio_cfg_data_rx, false, true, 32);
    sm_config_set_out_shift(&s->pio_cfg_data_rx, false, true, 32);
    sm_config_set_clkdiv(&s->pio_cfg_data_rx, clk_div);

    // Data TX program.
    s->pio_data_tx_offset = pio_add_program(s->pio, &sdio_data_tx_program);
    s->pio_cfg_data_tx = sdio_data_tx_program_get_default_config(s->pio_data_tx_offset);
    sm_config_set_in_pins(&s->pio_cfg_data_tx, d0_gpio);
    sm_config_set_set_pins(&s->pio_cfg_data_tx, d0_gpio, 4);
    sm_config_set_out_pins(&s->pio_cfg_data_tx, d0_gpio, 4);
    sm_config_set_in_shift(&s->pio_cfg_data_tx, false, false, 32);
    sm_config_set_out_shift(&s->pio_cfg_data_tx, false, true, 32);
    sm_config_set_clkdiv(&s->pio_cfg_data_tx, clk_div);

    // Disable input synchroniser to reduce latency (CLK is driven synchronously).
    s->pio->input_sync_bypass |=
        (1u << clk_gpio) | (1u << cmd_gpio) |
        (1u << d0_gpio) | (1u << (d0_gpio + 1)) |
        (1u << (d0_gpio + 2)) | (1u << (d0_gpio + 3));

    // Assign GPIO functions.
    gpio_function_t fn = (s->pio == pio1) ? GPIO_FUNC_PIO1 : GPIO_FUNC_PIO0;
    gpio_set_function(cmd_gpio,       fn);
    gpio_set_function(clk_gpio,       fn);
    gpio_set_function(d0_gpio,        fn);
    gpio_set_function(d0_gpio + 1,    fn);
    gpio_set_function(d0_gpio + 2,    fn);
    gpio_set_function(d0_gpio + 3,    fn);

    gpio_set_slew_rate(cmd_gpio,    GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(clk_gpio,    GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(d0_gpio,     GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(d0_gpio + 1, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(d0_gpio + 2, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(d0_gpio + 3, GPIO_SLEW_RATE_FAST);

    // Enable pull-ups on all SDIO lines (required by spec during init).
    gpio_pull_up(cmd_gpio);
    gpio_pull_up(d0_gpio);
    gpio_pull_up(d0_gpio + 1);
    gpio_pull_up(d0_gpio + 2);
    gpio_pull_up(d0_gpio + 3);

    s->transfer_state = SDIO_TRANSFER_IDLE;
    return true;
}

// ---------------------------------------------------------------------------
// Deinit
// ---------------------------------------------------------------------------

void rp2350_sdio_deinit(rp2350_sdio_state_t *s) {
    if (!s->resources_claimed) {
        return;
    }

    // Best-effort CMD0: reset the SD card to idle state before stopping the
    // clock.  This ensures the card is in a known idle state when CLK goes
    // away, preventing it from misinterpreting a floating CLK as phantom
    // clock edges and ending up in a confused state on the next init.
    {
        uint32_t word0 = (47u << 24) | (1u << 22); // CMD0, arg=0
        uint32_t word1 = (1u << 8);               // end-bit
        // CRC7(0x40,0,0,0,0) = 0x4A; crc|1 = 0x95
        word1 = (word1 & 0xFFFF00FFu) | ((uint32_t)(0x95u) << 8);
        pio_sm_clear_fifos(s->pio, s->cmd_sm);
        pio_sm_put(s->pio, s->cmd_sm, word0);
        pio_sm_put(s->pio, s->cmd_sm, word1);
        // Wait for the TX FIFO to drain (command sent) — max ~2ms at 400 kHz.
        uint64_t t = sdio_millis() + 5;
        while (pio_sm_get_tx_fifo_level(s->pio, s->cmd_sm) > 0 &&
               sdio_millis() < t) {
        }
        // Let CLK run a bit longer so the card fully receives CMD0.
        common_hal_time_delay_ms(2);
    }

    rp2350_sdio_stop(s);
    pio_sm_set_enabled(s->pio, s->cmd_sm, false);
    pio_sm_unclaim(s->pio, s->cmd_sm);
    pio_sm_unclaim(s->pio, s->data_sm);
    dma_channel_unclaim((uint)s->dma_ch);
    dma_channel_unclaim((uint)s->dma_chb);

    // Reset GPIO functions and direction to input (GPIO_FUNC_NULL keeps the
    // pin as GPIO but leaves output-enable set from PIO — call gpio_init() to
    // clear OE and return each pin to a safe floating input state).
    gpio_set_function(s->cmd_gpio,       GPIO_FUNC_NULL);
    gpio_set_function(s->clk_gpio,       GPIO_FUNC_NULL);
    gpio_set_function(s->d0_gpio,        GPIO_FUNC_NULL);
    gpio_set_function(s->d0_gpio + 1,    GPIO_FUNC_NULL);
    gpio_set_function(s->d0_gpio + 2,    GPIO_FUNC_NULL);
    gpio_set_function(s->d0_gpio + 3,    GPIO_FUNC_NULL);

    // Drive CLK LOW (not floating) to prevent the SD card from seeing phantom
    // clock edges due to capacitive coupling on an undriven pin.  The other
    // pins revert to safe floating inputs via gpio_init().
    gpio_init(s->cmd_gpio);
    gpio_set_dir(s->clk_gpio, true);   // output
    gpio_put(s->clk_gpio, 0);          // CLK LOW
    gpio_init(s->d0_gpio);
    gpio_init(s->d0_gpio + 1);
    gpio_init(s->d0_gpio + 2);
    gpio_init(s->d0_gpio + 3);

    s->resources_claimed = false;
}
