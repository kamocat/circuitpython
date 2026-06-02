// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 CircuitPython Contributors
//
// SPDX-License-Identifier: MIT
//
// RP2350 implementation of asdioio.ASDioCard — async SDIO block device.
// Uses DMA_IRQ_2 (RP2350 only) for IRQ-driven completion.

#pragma once

#include "py/obj.h"
#include "py/circuitpy_objawaitable.h"
#include "../sdioio/rp2350_sdio.h"

#if MICROPY_PY_ASYNC_AWAIT

// Per-object state for asdioio.ASDioCard.
// Layout mirrors sdioio_sdcard_obj_t so the same SD-init code can be reused.
typedef struct {
    mp_obj_base_t base;

    // PIO/DMA transport state (inline — same as sdioio).
    rp2350_sdio_state_t sdio;

    // SD card properties discovered during init.
    uint32_t sectors;       // total number of 512-byte blocks
    int cdv;                // block-address divisor: 1 (SDHC/SDXC) or 512 (v1)
    uint32_t frequency;     // actual clock frequency in Hz (after configure())
    uint8_t width;          // current bus width: 1 or 4
    uint16_t rca;           // relative card address (from CMD3)

    bool deinited;
    bool never_reset_flag;
} asdioio_asdiocard_obj_t;

// Per-transfer context.  One instance lives for the duration of a single
// async readblocks / writeblocks call.  Allocated via m_new_obj on the heap
// so the GC traces it for the lifetime of the awaitable.
typedef struct {
    asdioio_asdiocard_obj_t *card;      // back-pointer (GC-traced via tuple data)
    circuitpy_async_flag_t *flag;       // points into circuitpy_awaitable_obj_t
    uint8_t *buf;                       // current block position in caller buffer
    uint32_t block_addr;                // SD block address of current block
    uint32_t nblocks;                   // blocks remaining (counts down)
    uint32_t total_blocks;              // total blocks (for CMD12 decision)
    bool is_write;
} asdioio_ctx_t;

// ---------------------------------------------------------------------------
// API — called from shared-bindings/asdioio/ASDioCard.c
// ---------------------------------------------------------------------------

void common_hal_asdioio_asdiocard_construct(
    asdioio_asdiocard_obj_t *self,
    const mcu_pin_obj_t *command,
    const mcu_pin_obj_t *data0,
    uint32_t frequency);

void    common_hal_asdioio_asdiocard_deinit(asdioio_asdiocard_obj_t *self);
bool    common_hal_asdioio_asdiocard_deinited(asdioio_asdiocard_obj_t *self);
uint32_t common_hal_asdioio_asdiocard_get_count(asdioio_asdiocard_obj_t *self);
void    common_hal_asdioio_asdiocard_never_reset(asdioio_asdiocard_obj_t *self);

// Async transfer callbacks wired into circuitpy_awaitable_obj_t.
// data is a 3-tuple: (asdiocard_obj, start_block_int, buf_memoryview)
void *common_hal_asdioio_asdiocard_readblocks_start(circuitpy_async_flag_t *flag, mp_obj_t data);
mp_obj_t common_hal_asdioio_asdiocard_readblocks_end(void *ctx);
void     common_hal_asdioio_asdiocard_readblocks_cancel(void *ctx);

void *common_hal_asdioio_asdiocard_writeblocks_start(circuitpy_async_flag_t *flag, mp_obj_t data);
mp_obj_t common_hal_asdioio_asdiocard_writeblocks_end(void *ctx);
void     common_hal_asdioio_asdiocard_writeblocks_cancel(void *ctx);

// VFS native-path helpers (synchronous, used by extmod/vfs_blockdev.c).
mp_negative_errno_t asdioio_asdiocard_readblocks(mp_obj_t self_in, uint8_t *buf,
    uint32_t start_block, uint32_t buflen);
mp_negative_errno_t asdioio_asdiocard_writeblocks(mp_obj_t self_in, uint8_t *buf,
    uint32_t start_block, uint32_t buflen);
bool asdioio_asdiocard_ioctl(mp_obj_t self_in, size_t cmd, size_t arg,
    mp_int_t *out_value);

#endif // MICROPY_PY_ASYNC_AWAIT
