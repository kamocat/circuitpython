// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2026 CircuitPython Contributors
//
// SPDX-License-Identifier: MIT

#pragma once

#include "py/obj.h"
#include "py/circuitpy_objawaitable.h"

#include "common-hal/busio/SPI.h"
#include "common-hal/digitalio/DigitalInOut.h"
#include "shared-bindings/abusio/SPI.h"

#if MICROPY_PY_ASYNC_AWAIT

// Transfer phases for the per-block async state machine.
typedef enum {
    ASDCARD_PHASE_IDLE = 0,
    ASDCARD_PHASE_CMD,          // send command frame (CMD17/CMD18/CMD24/CMD25)
    ASDCARD_PHASE_TOKEN_WAIT,   // poll for 0xFE start-block token (read path)
    ASDCARD_PHASE_DMA,          // DMA in flight — awaitable is yielding
    ASDCARD_PHASE_CRC,          // DMA done — consume 2-byte CRC
    ASDCARD_PHASE_BUSY_WAIT,    // per-block programming — CS deasserted, bus unlocked
    ASDCARD_PHASE_STOP_WAIT,    // CMD25 STOP_TRAN sent — CS deasserted, bus unlocked
    ASDCARD_PHASE_DONE,         // all blocks transferred; send stop + deassert CS
} asdcard_phase_t;

// Per-object state.  abusio_spi_obj_t embeds busio_spi_obj_t as its first
// member so we can cast freely between the two for synchronous helper calls.
typedef struct {
    mp_obj_base_t base;
    abusio_spi_obj_t *bus;              // abusio.SPI instance (caller-owned)
    digitalio_digitalinout_obj_t cs;    // chip-select (owned by this object)
    int cdv;                            // block-address divisor: 512 (v1) or 1 (v2 SDHC)
    int baudrate;                       // working baudrate (250 kHz during init)
    uint32_t sectors;                   // total 512-byte blocks on card
    bool persistent_mount;              // survive VM resets (set by storage automount)
} asdcardio_asdcard_obj_t;

// Context allocated on the heap during an async readblocks / writeblocks call.
// One instance lives for the entire multi-block transfer; it is freed (GC'd)
// after the awaitable's end() / cancel() function runs.
typedef struct {
    asdcardio_asdcard_obj_t *card;      // back-pointer
    circuitpy_async_flag_t *flag;       // points into the circuitpy_awaitable_obj_t
    uint8_t *buf;                       // current write position in caller buffer
    uint32_t block;                     // SD card block address of next block
    uint32_t nblocks;                   // blocks remaining (counts down)
    uint32_t total_blocks;              // total blocks requested (for CMD18/CMD25)
    bool is_write;                      // true → write path, false → read path
    asdcard_phase_t phase;
    uint64_t busy_deadline;             // monotonic ns deadline for BUSY_WAIT / STOP_WAIT
    // Timing instrumentation (gated by TIMING_PRINT in ASdCard.c).
    uint64_t t_start;                   // monotonic ns at end of writeblocks_start()
    uint64_t blocking_ns;               // cumulative time spent in blocking sections
    uint32_t busy_polls;                // number of BUSY_WAIT / STOP_WAIT poll iterations
    // Sub-context passed to abusio for the current 512-byte DMA transfer.
    // We store the data tuple here so that GC can trace it.
    mp_obj_t abusio_data;               // tuple built for common_hal_abusio_spi_* calls
    void *abusio_ctx;                   // context returned by abusio start()
} asdcardio_transfer_ctx_t;

// Internal constructor called by shared-bindings make_new and by storage automount.
// Returns an mp_rom_error_text_t on failure, NULL on success.
mp_rom_error_text_t asdcardio_asdcard_construct(
    asdcardio_asdcard_obj_t *self,
    abusio_spi_obj_t *bus,
    const mcu_pin_obj_t *cs,
    int baudrate,
    bool persistent_mount);

// common_hal functions called from shared-bindings.
void common_hal_asdcardio_asdcard_construct(
    asdcardio_asdcard_obj_t *self,
    abusio_spi_obj_t *bus,
    const mcu_pin_obj_t *cs,
    int baudrate);

bool common_hal_asdcardio_asdcard_deinited(asdcardio_asdcard_obj_t *self);
void common_hal_asdcardio_asdcard_deinit(asdcardio_asdcard_obj_t *self);
uint32_t common_hal_asdcardio_asdcard_get_blockcount(asdcardio_asdcard_obj_t *self);
int common_hal_asdcardio_asdcard_sync(asdcardio_asdcard_obj_t *self);

// Async transfer callbacks — wired into circuitpy_awaitable_obj_t.
// data is a 3-tuple: (asdcard_obj, start_block_int, buf_memoryview)
void *common_hal_asdcardio_asdcard_readblocks_start(circuitpy_async_flag_t *flag, mp_obj_t data);
mp_obj_t common_hal_asdcardio_asdcard_readblocks_end(void *ctx);
void common_hal_asdcardio_asdcard_readblocks_cancel(void *ctx);

void *common_hal_asdcardio_asdcard_writeblocks_start(circuitpy_async_flag_t *flag, mp_obj_t data);
mp_obj_t common_hal_asdcardio_asdcard_writeblocks_end(void *ctx);
void common_hal_asdcardio_asdcard_writeblocks_cancel(void *ctx);

// VFS native-path functions (synchronous, used by extmod/vfs_blockdev.c).
mp_uint_t asdcardio_asdcard_readblocks(mp_obj_t self_in, uint8_t *buf, uint32_t start_block, uint32_t nblocks);
mp_uint_t asdcardio_asdcard_writeblocks(mp_obj_t self_in, uint8_t *buf, uint32_t start_block, uint32_t nblocks);
bool asdcardio_asdcard_ioctl(mp_obj_t self_in, size_t cmd, size_t arg, mp_int_t *out_value);

#endif // MICROPY_PY_ASYNC_AWAIT
