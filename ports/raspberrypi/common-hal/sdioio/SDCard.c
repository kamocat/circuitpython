// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 CircuitPython Contributors
//
// SPDX-License-Identifier: MIT
//
// RP2350 implementation of sdioio.SDCard using PIO + DMA SDIO.

#include <string.h>
#include "py/mperrno.h"
#include "py/runtime.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "shared-bindings/sdioio/SDCard.h"
#include "shared-bindings/time/__init__.h"
#include "supervisor/shared/translate/translate.h"

#include "SDCard.h"
#include "rp2350_sdio.h"

#include "hardware/clocks.h"
#include "extmod/vfs.h"

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

static inline uint64_t now_ms(void) {
    return common_hal_time_monotonic_ms();
}

// ---------------------------------------------------------------------------
// SD card commands and bit patterns
// ---------------------------------------------------------------------------

#define CMD0    0   // GO_IDLE_STATE
#define CMD2    2   // ALL_SEND_CID
#define CMD3    3   // SEND_RELATIVE_ADDR
#define CMD6    6   // SWITCH_FUNC
#define CMD7    7   // SELECT_CARD
#define CMD8    8   // SEND_IF_COND
#define CMD12   12  // STOP_TRANSMISSION
#define CMD16   16  // SET_BLOCKLEN
#define CMD17   17  // READ_SINGLE_BLOCK
#define CMD18   18  // READ_MULTIPLE_BLOCK
#define CMD24   24  // WRITE_BLOCK
#define CMD25   25  // WRITE_MULTIPLE_BLOCK
#define CMD55   55  // APP_CMD
#define CMD58   58  // READ_OCR (SPI mode only)
#define ACMD6   6   // SET_BUS_WIDTH (issued after CMD55)
#define ACMD41  41  // SD_SEND_OP_COND (issued after CMD55)

#define R1_IDLE_STATE       0x01u
#define R1_ILLEGAL_CMD      0x04u

#define OCR_BUSY            0x80000000u
#define OCR_SDHC            0x40000000u
#define OCR_VDD_33          0x00300000u   // 3.2–3.4 V window

// ---------------------------------------------------------------------------
// Helper: send ACMD (CMD55 prefix + command)
// ---------------------------------------------------------------------------

static sdio_status_t send_acmd(sdioio_sdcard_obj_t *self,
    uint8_t cmd, uint32_t arg, uint32_t *response) {
    uint32_t r1 = 0;
    sdio_status_t st = rp2350_sdio_command_R1(&self->sdio, CMD55,
        (uint32_t)self->rca << 16, &r1);
    if (st != SDIO_OK) {
        return st;
    }
    // After card init, RCA is 0 for CMD55; ignore R1 value during init.
    return rp2350_sdio_command_R1(&self->sdio, cmd, arg, response);
}

// ---------------------------------------------------------------------------
// Card initialisation sequence
// ---------------------------------------------------------------------------

static int sdioio_init_card(sdioio_sdcard_obj_t *self) {
    sdio_status_t st;
    uint32_t resp = 0;

    // CMD0 — reset to idle.
    rp2350_sdio_command_R1(&self->sdio, CMD0, 0, NULL);
    common_hal_time_delay_ms(2);

    // CMD8 — voltage check (VHS = 0x1, check pattern 0xAA).
    st = rp2350_sdio_command_R1(&self->sdio, CMD8, 0x000001AAu, &resp);
    bool is_v2 = (st == SDIO_OK);
    if (is_v2 && (resp & 0xFFF) != 0x1AA) {
        // Voltage mismatch — card not supported.
        return -MP_EIO;
    }

    // ACMD41 — send OCR, wait for card to leave idle.
    uint32_t arg41 = OCR_VDD_33;
    if (is_v2) {
        arg41 |= OCR_SDHC; // HCS=1: host supports SDHC/SDXC
    }
    uint64_t deadline = now_ms() + 2000;
    do {
        st = send_acmd(self, ACMD41, arg41, &resp);
        if (st != SDIO_OK) {
            return -MP_EIO;
        }
        if (now_ms() > deadline) {
            return -MP_ETIMEDOUT;
        }
    } while (!(resp & OCR_BUSY));

    // Determine card type.
    self->cdv = (is_v2 && (resp & OCR_SDHC)) ? 1 : 512;

    // CMD2 — get CID (136-bit response; we ignore the payload).
    uint8_t cid[16];
    st = rp2350_sdio_command_R2(&self->sdio, CMD2, 0, cid);
    if (st != SDIO_OK) {
        return -MP_EIO;
    }

    // CMD3 — get relative card address.
    st = rp2350_sdio_command_R1(&self->sdio, CMD3, 0, &resp);
    if (st != SDIO_OK) {
        return -MP_EIO;
    }
    self->rca = (uint16_t)(resp >> 16);

    // CMD7 — select card using RCA.
    st = rp2350_sdio_command_R1(&self->sdio, CMD7, (uint32_t)self->rca << 16, &resp);
    if (st != SDIO_OK) {
        return -MP_EIO;
    }

    // ACMD6 — switch to 4-bit bus width.
    if (self->width == 4) {
        st = send_acmd(self, ACMD6, 0x2, &resp); // bus width = 4
        if (st != SDIO_OK) {
            // Fall back to 1-bit.
            self->width = 1;
        }
    }

    // CMD16 — set block length to 512.
    st = rp2350_sdio_command_R1(&self->sdio, CMD16, RP2350_SDIO_BLOCK_SIZE, &resp);
    if (st != SDIO_OK) {
        return -MP_EIO;
    }

    // Read CSD (CMD9) to determine card capacity.
    uint8_t csd[16];
    st = rp2350_sdio_command_R2(&self->sdio, 9, (uint32_t)self->rca << 16, csd);
    if (st != SDIO_OK) {
        return -MP_EIO;
    }

    // Decode sector count from CSD.
    uint8_t csd_version = (csd[0] >> 6) & 3;
    if (csd_version == 1) {
        // CSD v2 (SDHC/SDXC): sectors = (C_SIZE + 1) * 1024
        uint32_t c_size = (((uint32_t)csd[7] & 0x3F) << 16) |
            ((uint32_t)csd[8] << 8) |
            (uint32_t)csd[9];
        self->sectors = (c_size + 1) * 1024u;
    } else {
        // CSD v1
        uint32_t read_bl_len = csd[5] & 0x0F;
        uint32_t c_size =
            (((uint32_t)csd[6] & 0x03) << 10) |
            ((uint32_t)csd[7] << 2) |
            ((uint32_t)csd[8] >> 6);
        uint32_t c_size_mult =
            (((uint32_t)csd[9] & 0x03) << 1) |
            ((uint32_t)csd[10] >> 7);
        uint32_t block_size = 1u << read_bl_len;
        uint32_t block_count = (c_size + 1) * (1u << (c_size_mult + 2));
        self->sectors = block_count * (block_size / RP2350_SDIO_BLOCK_SIZE);
    }

    return 0;
}

// ---------------------------------------------------------------------------
// common_hal interface
// ---------------------------------------------------------------------------

void common_hal_sdioio_sdcard_construct(sdioio_sdcard_obj_t *self,
    const mcu_pin_obj_t *clock, const mcu_pin_obj_t *command,
    uint8_t num_data, const mcu_pin_obj_t **data, uint32_t frequency) {

    memset(&self->sdio, 0, sizeof(self->sdio));
    self->deinited = false;
    self->never_reset_flag = false;
    self->width = (num_data >= 4) ? 4 : 1;

    uint d0_gpio = data[0]->number;
    uint cmd_gpio = command->number;
    // CLK must be exactly (D0 - 2) mod 32 per PIO program constraint.
    uint clk_gpio = (d0_gpio + 32 - SDIO_CLK_PIN_D0_OFFSET) % 32;
    if (clock->number != clk_gpio) {
        mp_raise_ValueError_varg(
            MP_ERROR_TEXT("CLK pin must be %d (D0-2 mod 32)"), clk_gpio);
    }
    // D1-D3 must be consecutive.
    if (num_data >= 4) {
        for (int i = 1; i < 4; i++) {
            if (data[i]->number != d0_gpio + (uint)i) {
                mp_raise_ValueError(MP_ERROR_TEXT("Data pins must be consecutive D0, D0+1, D0+2, D0+3"));
            }
        }
    }

    // Compute PIO clock divider.
    uint32_t sys_hz = clock_get_hz(clk_sys);
    float clk_div = (float)sys_hz / ((float)CLKDIV * (float)frequency);
    if (clk_div < 1.0f) {
        clk_div = 1.0f;
    }
    // Store effective frequency.
    self->frequency = (uint32_t)((float)sys_hz / ((float)CLKDIV * clk_div));

    // Claim pins.
    claim_pin(clock);
    claim_pin(command);
    for (int i = 0; i < num_data; i++) {
        claim_pin(data[i]);
    }

    // Slow start: initialise at 400 kHz (per SD spec).
    float init_div = (float)sys_hz / ((float)CLKDIV * 400000.0f);
    if (!rp2350_sdio_init(&self->sdio, clk_gpio, cmd_gpio, d0_gpio, init_div)) {
        mp_raise_OSError(MP_EIO);
    }

    // Card initialisation sequence.
    int result = sdioio_init_card(self);
    if (result < 0) {
        rp2350_sdio_deinit(&self->sdio);
        mp_raise_OSError(-result);
    }

    // Switch to requested clock speed.
    rp2350_sdio_set_clkdiv(&self->sdio, clk_div);
}

void common_hal_sdioio_sdcard_deinit(sdioio_sdcard_obj_t *self) {
    if (self->deinited) {
        return;
    }
    rp2350_sdio_deinit(&self->sdio);
    self->deinited = true;
}

bool common_hal_sdioio_sdcard_deinited(sdioio_sdcard_obj_t *self) {
    return self->deinited;
}

bool common_hal_sdioio_sdcard_configure(sdioio_sdcard_obj_t *self,
    uint32_t baudrate, uint8_t width) {
    if (baudrate != 0) {
        uint32_t sys_hz = clock_get_hz(clk_sys);
        float clk_div = (float)sys_hz / ((float)CLKDIV * (float)baudrate);
        if (clk_div < 1.0f) {
            clk_div = 1.0f;
        }
        self->frequency = (uint32_t)((float)sys_hz / ((float)CLKDIV * clk_div));
        rp2350_sdio_set_clkdiv(&self->sdio, clk_div);
    }
    if (width != 0) {
        self->width = width;
    }
    return true;
}

void common_hal_sdioio_sdcard_unlock(sdioio_sdcard_obj_t *self) {
    // No locking needed for SDIO (single owner).
}

uint32_t common_hal_sdioio_sdcard_get_frequency(sdioio_sdcard_obj_t *self) {
    return self->frequency;
}

uint8_t common_hal_sdioio_sdcard_get_width(sdioio_sdcard_obj_t *self) {
    return self->width;
}

uint32_t common_hal_sdioio_sdcard_get_count(sdioio_sdcard_obj_t *self) {
    return self->sectors;
}

void common_hal_sdioio_sdcard_never_reset(sdioio_sdcard_obj_t *self) {
    self->never_reset_flag = true;
}

// ---------------------------------------------------------------------------
// Block I/O
// ---------------------------------------------------------------------------

mp_negative_errno_t common_hal_sdioio_sdcard_readblocks(
    sdioio_sdcard_obj_t *self, uint32_t start_block, mp_buffer_info_t *bufinfo) {
    if (self->deinited) {
        return -MP_ENODEV;
    }
    uint32_t num_blocks = (uint32_t)(bufinfo->len / RP2350_SDIO_BLOCK_SIZE);
    if (num_blocks == 0 || bufinfo->len % RP2350_SDIO_BLOCK_SIZE != 0) {
        return -MP_EINVAL;
    }
    if (num_blocks > RP2350_SDIO_MAX_BLOCKS) {
        return -MP_EINVAL;
    }

    uint32_t addr = (self->cdv == 1) ? start_block : start_block * RP2350_SDIO_BLOCK_SIZE;
    uint32_t resp = 0;
    uint8_t cmd = (num_blocks == 1) ? CMD17 : CMD18;
    sdio_status_t st = rp2350_sdio_command_R1(&self->sdio, cmd, addr, &resp);
    if (st != SDIO_OK) {
        return -MP_EIO;
    }

    st = rp2350_sdio_rx_start(&self->sdio, (uint8_t *)bufinfo->buf, num_blocks);
    if (st != SDIO_OK) {
        return -MP_EIO;
    }

    // Poll until complete.
    while ((st = rp2350_sdio_rx_poll(&self->sdio)) == SDIO_BUSY) {
        RUN_BACKGROUND_TASKS;
    }

    if (num_blocks > 1) {
        // Send CMD12 to stop transmission.
        rp2350_sdio_command_R1(&self->sdio, CMD12, 0, &resp);
    }

    if (st != SDIO_OK) {
        return (st == SDIO_ERR_DATA_CRC) ? -MP_EIO : -MP_ETIMEDOUT;
    }
    return 0;
}

mp_negative_errno_t common_hal_sdioio_sdcard_writeblocks(
    sdioio_sdcard_obj_t *self, uint32_t start_block, mp_buffer_info_t *bufinfo) {
    if (self->deinited) {
        return -MP_ENODEV;
    }
    uint32_t num_blocks = (uint32_t)(bufinfo->len / RP2350_SDIO_BLOCK_SIZE);
    if (num_blocks == 0 || bufinfo->len % RP2350_SDIO_BLOCK_SIZE != 0) {
        return -MP_EINVAL;
    }
    if (num_blocks > RP2350_SDIO_MAX_BLOCKS) {
        return -MP_EINVAL;
    }

    uint32_t addr = (self->cdv == 1) ? start_block : start_block * RP2350_SDIO_BLOCK_SIZE;
    uint32_t resp = 0;
    uint8_t cmd = (num_blocks == 1) ? CMD24 : CMD25;
    sdio_status_t st = rp2350_sdio_command_R1(&self->sdio, cmd, addr, &resp);
    if (st != SDIO_OK) {
        return -MP_EIO;
    }

    st = rp2350_sdio_tx_start(&self->sdio, (const uint8_t *)bufinfo->buf, num_blocks);
    if (st != SDIO_OK) {
        return -MP_EIO;
    }

    while ((st = rp2350_sdio_tx_poll(&self->sdio)) == SDIO_BUSY) {
        RUN_BACKGROUND_TASKS;
    }

    if (num_blocks > 1) {
        rp2350_sdio_command_R1(&self->sdio, CMD12, 0, &resp);
    }

    if (st == SDIO_ERR_WRITE_CRC || st == SDIO_ERR_WRITE_FAIL) {
        return -MP_EIO;
    }
    if (st == SDIO_ERR_DATA_TIMEOUT) {
        return -MP_ETIMEDOUT;
    }
    return (st == SDIO_OK) ? 0 : -MP_EIO;
}

// ---------------------------------------------------------------------------
// Native VFS blockdev helpers
// ---------------------------------------------------------------------------

mp_negative_errno_t sdioio_sdcard_readblocks(mp_obj_t self_in, uint8_t *buf,
    uint32_t start_block, uint32_t buflen) {
    sdioio_sdcard_obj_t *self = (sdioio_sdcard_obj_t *)self_in;
    mp_buffer_info_t info = {.buf = buf, .len = buflen};
    return common_hal_sdioio_sdcard_readblocks(self, start_block, &info);
}

mp_negative_errno_t sdioio_sdcard_writeblocks(mp_obj_t self_in, uint8_t *buf,
    uint32_t start_block, uint32_t buflen) {
    sdioio_sdcard_obj_t *self = (sdioio_sdcard_obj_t *)self_in;
    mp_buffer_info_t info = {.buf = (void *)buf, .len = buflen};
    return common_hal_sdioio_sdcard_writeblocks(self, start_block, &info);
}

bool sdioio_sdcard_ioctl(mp_obj_t self_in, size_t cmd, size_t arg,
    mp_int_t *out_value) {
    sdioio_sdcard_obj_t *self = (sdioio_sdcard_obj_t *)self_in;
    switch (cmd) {
        case MP_BLOCKDEV_IOCTL_INIT:
            *out_value = 0;
            return true;
        case MP_BLOCKDEV_IOCTL_DEINIT:
            *out_value = 0;
            return true;
        case MP_BLOCKDEV_IOCTL_SYNC:
            *out_value = 0;
            return true;
        case MP_BLOCKDEV_IOCTL_BLOCK_COUNT:
            *out_value = (mp_int_t)self->sectors;
            return true;
        case MP_BLOCKDEV_IOCTL_BLOCK_SIZE:
            *out_value = RP2350_SDIO_BLOCK_SIZE;
            return true;
        default:
            return false;
    }
}
