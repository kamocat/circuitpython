// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2026 CircuitPython Contributors
//
// SPDX-License-Identifier: MIT

// Async SD card block device using abusio.SPI for DMA-backed transfers.
//
// This implementation mirrors shared-module/sdcardio/SDCard.c but replaces
// the blocking SPI data phase with a circuitpy_awaitable_obj_t so that the
// asyncio event loop can run other tasks while each 512-byte block is
// transferred by DMA.
//
// SD protocol references used:
//  - Physical Layer Simplified Specification v8.00
//  - adafruit_sdcard.py (pure-Python reference)
//  - shared-module/sdcardio/SDCard.c (the synchronous CircuitPython implementation)

#include "shared-module/asdcardio/ASdCard.h"

#include "extmod/vfs.h"
#include "shared-bindings/abusio/SPI.h"
#include "shared-bindings/busio/SPI.h"
#include "shared-bindings/digitalio/DigitalInOut.h"
#include "shared-bindings/asdcardio/ASdCard.h"
#include "shared-bindings/time/__init__.h"
#include "shared-bindings/util.h"
#include "py/mperrno.h"
#include "py/objtuple.h"
#include "py/objarray.h"

#if MICROPY_PY_ASYNC_AWAIT

// ---- Debug ------------------------------------------------------------------

#if 0
#define DEBUG_PRINT(...) ((void)mp_printf(&mp_plat_print,##__VA_ARGS__))
#else
#define DEBUG_PRINT(...) ((void)0)
#endif

// Enable to print per-transfer blocking/total time on completion.
// blocking_ns = time CPU was busy in writeblocks_start() + each end() invocation.
// Total time includes DMA transfers and BUSY_WAIT yields (bus released).
#if 0
#define TIMING_PRINT(...) ((void)mp_printf(&mp_plat_print,##__VA_ARGS__))
#else
#define TIMING_PRINT(...) ((void)0)
#endif

// ---- SD protocol constants --------------------------------------------------

#define CMD_TIMEOUT         (200)
#define READY_TIMEOUT_NS    (300 * 1000 * 1000)  // 300 ms
#define WRITE_TIMEOUT_NS    (1000 * 1000 * 1000) // 1 s — max card busy time

#define R1_IDLE_STATE       (1 << 0)
#define R1_ILLEGAL_COMMAND  (1 << 2)

#define TOKEN_CMD25         (0xFC)
#define TOKEN_STOP_TRAN     (0xFD)
#define TOKEN_DATA          (0xFE)

// ---- Bus cast helpers -------------------------------------------------------
//
// abusio_spi_obj_t embeds busio_spi_obj_t as its first member, so a cast
// between the two pointer types is always safe.

static inline busio_spi_obj_t *as_busio(asdcardio_asdcard_obj_t *self) {
    return &self->bus->spi;
}

// ---- Deinit helpers ---------------------------------------------------------

bool common_hal_asdcardio_asdcard_deinited(asdcardio_asdcard_obj_t *self) {
    return !self->bus || common_hal_busio_spi_deinited(as_busio(self));
}

static void check_for_deinit(asdcardio_asdcard_obj_t *self) {
    if (common_hal_asdcardio_asdcard_deinited(self)) {
        raise_deinited_error();
    }
}

// ---- CRC7 (command frames) --------------------------------------------------

static uint8_t CRC7(const uint8_t *data, uint8_t n) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < n; i++) {
        uint8_t d = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc <<= 1;
            if ((d & 0x80) ^ (crc & 0x80)) {
                crc ^= 0x09;
            }
            d <<= 1;
        }
    }
    return (crc << 1) | 1;
}

// ---- Low-level bus helpers (all synchronous, used during init & protocol) ---

static void cs_assert(asdcardio_asdcard_obj_t *self) {
    common_hal_digitalio_digitalinout_set_value(&self->cs, false);
}

static void cs_deassert(asdcardio_asdcard_obj_t *self) {
    common_hal_digitalio_digitalinout_set_value(&self->cs, true);
}

static bool lock_and_configure_bus(asdcardio_asdcard_obj_t *self) {
    if (common_hal_asdcardio_asdcard_deinited(self)) {
        return false;
    }
    if (!common_hal_busio_spi_try_lock(as_busio(self))) {
        return false;
    }
    if (!vm_is_running() && !self->persistent_mount) {
        common_hal_busio_spi_unlock(as_busio(self));
        return false;
    }
    common_hal_busio_spi_configure(as_busio(self), self->baudrate, 0, 0, 8);
    cs_assert(self);
    return true;
}

static void lock_bus_or_throw(asdcardio_asdcard_obj_t *self) {
    if (!lock_and_configure_bus(self)) {
        mp_raise_OSError(EAGAIN);
    }
}

static void clock_card(asdcardio_asdcard_obj_t *self, int bytes) {
    uint8_t buf = 0xff;
    cs_deassert(self);
    for (int i = 0; i < bytes; i++) {
        common_hal_busio_spi_write(as_busio(self), &buf, 1);
    }
}

static void extraclock_and_unlock_bus(asdcardio_asdcard_obj_t *self) {
    clock_card(self, 1);
    common_hal_busio_spi_unlock(as_busio(self));
}

static int wait_for_ready(asdcardio_asdcard_obj_t *self) {
    uint64_t deadline = common_hal_time_monotonic_ns() + READY_TIMEOUT_NS;
    uint8_t b;
    while (common_hal_time_monotonic_ns() < deadline) {
        common_hal_busio_spi_read(as_busio(self), &b, 1, 0xff);
        if (b == 0xff) {
            return 0;
        }
    }
    return -ETIMEDOUT;
}

// Generic SD command; returns R1 byte or negative errno.
// wait=true → call wait_for_ready before sending command frame.
// data_block=true → after R1, poll for 0xFE then read response_len bytes + 2 CRC bytes.
static int sd_cmd(asdcardio_asdcard_obj_t *self,
    int cmd_num, int arg,
    void *response_buf, size_t response_len,
    bool data_block, bool wait) {

    DEBUG_PRINT("cmd %3d arg=%08x len=%d\n", cmd_num, arg, (int)response_len);

    uint8_t cmdbuf[6];
    cmdbuf[0] = cmd_num | 0x40;
    cmdbuf[1] = (arg >> 24) & 0xff;
    cmdbuf[2] = (arg >> 16) & 0xff;
    cmdbuf[3] = (arg >> 8) & 0xff;
    cmdbuf[4] = arg & 0xff;
    cmdbuf[5] = CRC7(cmdbuf, 5);

    if (wait) {
        int r = wait_for_ready(self);
        if (r < 0) {
            return r;
        }
    }

    common_hal_busio_spi_write(as_busio(self), cmdbuf, 6);

    // Wait for response (MSB cleared)
    bool received = false;
    for (int i = 0; i < CMD_TIMEOUT; i++) {
        common_hal_busio_spi_read(as_busio(self), cmdbuf, 1, 0xff);
        if ((cmdbuf[0] & 0x80) == 0) {
            received = true;
            break;
        }
    }
    if (!received) {
        return -MP_EIO;
    }

    if (response_buf) {
        if (data_block) {
            uint8_t token = 0;
            do {
                common_hal_busio_spi_read(as_busio(self), &token, 1, 0xff);
            } while (token != TOKEN_DATA);
        }
        common_hal_busio_spi_read(as_busio(self), response_buf, response_len, 0xff);
        if (data_block) {
            // discard 2-byte CRC
            uint8_t crc[2];
            common_hal_busio_spi_read(as_busio(self), crc, 2, 0xff);
        }
    }

    return cmdbuf[0]; // R1
}

static int block_cmd(asdcardio_asdcard_obj_t *self,
    int cmd_num, uint32_t block,
    void *response_buf, size_t response_len,
    bool data_block, bool wait) {
    return sd_cmd(self, cmd_num, block * self->cdv, response_buf, response_len, data_block, wait);
}

// ---- Card initialisation (synchronous, done once in construct()) ------------

static mp_rom_error_text_t init_card_v1(asdcardio_asdcard_obj_t *self) {
    for (int i = 0; i < CMD_TIMEOUT; i++) {
        if (sd_cmd(self, 41, 0, NULL, 0, true, true) == 0) {
            return NULL;
        }
    }
    return MP_ERROR_TEXT("timeout waiting for v1 card");
}

static mp_rom_error_text_t init_card_v2(asdcardio_asdcard_obj_t *self) {
    for (int i = 0; i < CMD_TIMEOUT; i++) {
        uint8_t ocr[4];
        common_hal_time_delay_ms(50);
        sd_cmd(self, 58, 0, ocr, sizeof(ocr), false, true);
        sd_cmd(self, 55, 0, NULL, 0, true, true);
        if (sd_cmd(self, 41, 0x40000000, NULL, 0, true, true) == 0) {
            sd_cmd(self, 58, 0, ocr, sizeof(ocr), false, true);
            if ((ocr[0] & 0x40) != 0) {
                self->cdv = 1; // SDHC/SDXC — block-addressed
            }
            return NULL;
        }
    }
    return MP_ERROR_TEXT("timeout waiting for v2 card");
}

static mp_rom_error_text_t init_card(asdcardio_asdcard_obj_t *self) {
    clock_card(self, 10);
    cs_assert(self);

    // CMD0 — go to idle state (SPI mode)
    {
        bool idle = false;
        for (int i = 0; i < 5; i++) {
            (void)wait_for_ready(self);
            if (sd_cmd(self, 0, 0, NULL, 0, true, false) == R1_IDLE_STATE) {
                idle = true;
                break;
            }
        }
        if (!idle) {
            return MP_ERROR_TEXT("no SD card");
        }
    }

    // CMD8 — determine card version
    {
        uint8_t rb7[4];
        int resp = sd_cmd(self, 8, 0x1AA, rb7, sizeof(rb7), false, true);
        if (resp == R1_IDLE_STATE) {
            mp_rom_error_text_t r = init_card_v2(self);
            if (r != NULL) {
                return r;
            }
        } else if (resp == (R1_IDLE_STATE | R1_ILLEGAL_COMMAND)) {
            mp_rom_error_text_t r = init_card_v1(self);
            if (r != NULL) {
                return r;
            }
        } else {
            return MP_ERROR_TEXT("couldn't determine SD card version");
        }
    }

    // CMD9 — read CSD to get sector count
    {
        uint8_t csd[16];
        if (sd_cmd(self, 9, 0, csd, sizeof(csd), true, true) != 0) {
            return MP_ERROR_TEXT("no response from SD card");
        }
        int csd_version = (csd[0] & 0xC0) >> 6;
        if (csd_version >= 2) {
            return MP_ERROR_TEXT("SD card CSD format not supported");
        }
        if (csd_version == 1) {
            self->sectors = ((csd[8] << 8 | csd[9]) + 1) * 1024;
        } else {
            uint32_t block_length = 1 << (csd[5] & 0xF);
            uint32_t c_size = ((csd[6] & 0x3) << 10) | (csd[7] << 2) | ((csd[8] & 0xC) >> 6);
            uint32_t mult = 1 << (((csd[9] & 0x3) << 1 | (csd[10] & 0x80) >> 7) + 2);
            self->sectors = block_length / 512 * mult * (c_size + 1);
        }
    }

    // CMD16 — set block length to 512
    if (sd_cmd(self, 16, 512, NULL, 0, true, true) != 0) {
        return MP_ERROR_TEXT("can't set 512 block size");
    }

    return NULL;
}

// ---- construct / deinit / count / sync --------------------------------------

mp_rom_error_text_t asdcardio_asdcard_construct(
    asdcardio_asdcard_obj_t *self,
    abusio_spi_obj_t *bus,
    const mcu_pin_obj_t *cs_pin,
    int baudrate,
    bool persistent_mount) {

    self->bus = bus;
    self->persistent_mount = persistent_mount;
    self->cdv = 512;
    self->sectors = 0;
    self->baudrate = 250000; // slow during init

    common_hal_digitalio_digitalinout_construct(&self->cs, cs_pin);
    common_hal_digitalio_digitalinout_switch_to_output(&self->cs, true, DRIVE_MODE_PUSH_PULL);

    lock_bus_or_throw(self);
    mp_rom_error_text_t result = init_card(self);
    extraclock_and_unlock_bus(self);

    if (result != NULL) {
        common_hal_digitalio_digitalinout_deinit(&self->cs);
        return result;
    }

    self->baudrate = baudrate;
    return NULL;
}

void common_hal_asdcardio_asdcard_construct(
    asdcardio_asdcard_obj_t *self,
    abusio_spi_obj_t *bus,
    const mcu_pin_obj_t *cs_pin,
    int baudrate) {
    mp_rom_error_text_t result = asdcardio_asdcard_construct(self, bus, cs_pin, baudrate, false);
    if (result != NULL) {
        mp_raise_OSError_msg(result);
    }
}

void common_hal_asdcardio_asdcard_deinit(asdcardio_asdcard_obj_t *self) {
    if (common_hal_asdcardio_asdcard_deinited(self)) {
        return;
    }
    common_hal_asdcardio_asdcard_sync(self);
    self->bus = NULL;
    common_hal_digitalio_digitalinout_deinit(&self->cs);
}

uint32_t common_hal_asdcardio_asdcard_get_blockcount(asdcardio_asdcard_obj_t *self) {
    check_for_deinit(self);
    return self->sectors;
}

int common_hal_asdcardio_asdcard_sync(asdcardio_asdcard_obj_t *self) {
    // Nothing to flush for reads; writes are already committed per-block.
    // We still lock/unlock to ensure any in-progress CMD25 is cleanly terminated
    // (asdcardio uses CMD24 per-block for writes, so in_cmd25 state is not tracked,
    // but a bus lock/unlock still serves as a memory barrier).
    if (!lock_and_configure_bus(self)) {
        return -EAGAIN;
    }
    extraclock_and_unlock_bus(self);
    return 0;
}

// ---- Synchronous block I/O (used by VFS native path) ------------------------

static int sync_readinto(asdcardio_asdcard_obj_t *self, void *buf) {
    uint8_t token = 0;
    // Wait for start-block token
    while (token != TOKEN_DATA) {
        common_hal_busio_spi_read(as_busio(self), &token, 1, 0xff);
    }
    common_hal_busio_spi_read(as_busio(self), buf, 512, 0xff);
    // Discard 2-byte CRC
    uint8_t crc[2];
    common_hal_busio_spi_read(as_busio(self), crc, 2, 0xff);
    return 0;
}

static int sync_write(asdcardio_asdcard_obj_t *self, uint8_t token, const void *buf) {
    wait_for_ready(self);
    common_hal_busio_spi_write(as_busio(self), &token, 1);
    common_hal_busio_spi_write(as_busio(self), buf, 512);
    // 2 dummy CRC bytes
    uint8_t crc[2] = {0xff, 0xff};
    common_hal_busio_spi_write(as_busio(self), crc, 2);
    // Check data response
    uint8_t resp = 0;
    for (int i = 0; i < CMD_TIMEOUT; i++) {
        common_hal_busio_spi_read(as_busio(self), &resp, 1, 0xff);
        if ((resp & 0x11) == 0x01) {
            break;
        }
    }
    if ((resp & 0x1f) != 0x05) {
        return -MP_EIO;
    }
    // Wait for write to finish (1 s timeout)
    uint64_t deadline = common_hal_time_monotonic_ns() + WRITE_TIMEOUT_NS;
    uint8_t b = 0;
    do {
        common_hal_busio_spi_read(as_busio(self), &b, 1, 0xff);
        if (common_hal_time_monotonic_ns() > deadline) {
            return -MP_ETIMEDOUT;
        }
    } while (b == 0);
    return 0;
}

mp_uint_t asdcardio_asdcard_readblocks(mp_obj_t self_in, uint8_t *buf, uint32_t start_block, uint32_t nblocks) {
    asdcardio_asdcard_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (!lock_and_configure_bus(self)) {
        return MP_EAGAIN;
    }
    int r = 0;
    if (nblocks == 1) {
        r = block_cmd(self, 17, start_block, buf, 512, true, true);
    } else {
        r = block_cmd(self, 18, start_block, NULL, 0, false, true);
        uint8_t *ptr = buf;
        while (nblocks-- && r >= 0) {
            r = sync_readinto(self, ptr);
            ptr += 512;
        }
        // CMD12 — stop transmission
        r = sd_cmd(self, 12, 0, NULL, 0, true, false);
        uint8_t b;
        while (r != 0) {
            common_hal_busio_spi_read(as_busio(self), &b, 1, 0xff);
            if (b & 0x80) {
                break;
            }
            r = b;
        }
    }
    extraclock_and_unlock_bus(self);
    return r < 0 ? (mp_uint_t)(-r) : 0;
}

mp_uint_t asdcardio_asdcard_writeblocks(mp_obj_t self_in, uint8_t *buf, uint32_t start_block, uint32_t nblocks) {
    asdcardio_asdcard_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (!lock_and_configure_bus(self)) {
        return MP_EAGAIN;
    }
    // Use CMD24 (single-block write) for each block — simpler state machine
    // compared to CMD25, and correctness is more important here.
    uint8_t *ptr = buf;
    int r = 0;
    for (uint32_t i = 0; i < nblocks; i++) {
        r = block_cmd(self, 24, start_block + i, NULL, 0, false, true);
        if (r < 0) {
            break;
        }
        r = sync_write(self, TOKEN_DATA, ptr);
        if (r < 0) {
            break;
        }
        ptr += 512;
    }
    extraclock_and_unlock_bus(self);
    return r < 0 ? (mp_uint_t)(-r) : 0;
}

bool asdcardio_asdcard_ioctl(mp_obj_t self_in, size_t cmd, size_t arg, mp_int_t *out_value) {
    asdcardio_asdcard_obj_t *self = MP_OBJ_TO_PTR(self_in);
    *out_value = 0;
    switch (cmd) {
        case MP_BLOCKDEV_IOCTL_DEINIT:
        case MP_BLOCKDEV_IOCTL_SYNC:
            return common_hal_asdcardio_asdcard_sync(self) == 0;
        case MP_BLOCKDEV_IOCTL_BLOCK_COUNT:
            *out_value = (mp_int_t)self->sectors;
            return true;
        case MP_BLOCKDEV_IOCTL_BLOCK_SIZE:
            *out_value = 512;
            return true;
        default:
            return false;
    }
}

// ---- Async transfer helpers -------------------------------------------------
//
// The circuitpy_awaitable_obj_t polls its flag each time the event loop calls
// iternext().  For multi-block transfers we need N DMA completions, each
// signalled through the same flag.  We achieve this by re-arming the DMA
// inside readblocks_end() / writeblocks_end() and returning MP_OBJ_NULL to
// signal to awaitable_iternext() "not done yet — re-queue and poll again".
// See the corresponding patch in py/circuitpy_objawaitable.c.

// Build a 3-tuple (abusio_spi_obj, buf_memoryview, write_value) suitable for
// passing to common_hal_abusio_spi_readinto_start.
static mp_obj_t make_readinto_data(asdcardio_asdcard_obj_t *card, uint8_t *buf) {
    mp_obj_t spi_obj = MP_OBJ_FROM_PTR(card->bus);
    mp_obj_t mv = mp_obj_new_memoryview(
        'B' | MP_OBJ_ARRAY_TYPECODE_FLAG_RW, 512, buf);
    mp_obj_t items[3] = { spi_obj, mv, mp_obj_new_int(0xff) };
    return mp_obj_new_tuple(3, items);
}

// Build a 2-tuple (abusio_spi_obj, buf_memoryview) suitable for
// common_hal_abusio_spi_write_start.
static mp_obj_t make_write_data(asdcardio_asdcard_obj_t *card, const uint8_t *buf) {
    mp_obj_t spi_obj = MP_OBJ_FROM_PTR(card->bus);
    // Cast away const — the memoryview is read-only ('B' without RW flag).
    mp_obj_t mv = mp_obj_new_memoryview('B', 512, (void *)buf);
    mp_obj_t items[2] = { spi_obj, mv };
    return mp_obj_new_tuple(2, items);
}

// ---- readblocks async -------------------------------------------------------

void *common_hal_asdcardio_asdcard_readblocks_start(circuitpy_async_flag_t *flag, mp_obj_t data) {
    // data is a 3-tuple: (asdcard_obj, start_block_int, buf_memoryview)
    mp_obj_t *items;
    size_t len;
    mp_obj_tuple_get(data, &len, &items);
    // len must be 3
    asdcardio_asdcard_obj_t *card = MP_OBJ_TO_PTR(items[0]);
    uint32_t start_block = (uint32_t)mp_obj_get_int(items[1]);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(items[2], &bufinfo, MP_BUFFER_WRITE);

    uint32_t nblocks = bufinfo.len / 512;

    // Acquire bus (must succeed — caller validated before constructing awaitable)
    if (!lock_and_configure_bus(card)) {
        mp_raise_OSError(EAGAIN);
    }

    asdcardio_transfer_ctx_t *ctx = m_new_obj(asdcardio_transfer_ctx_t);
    ctx->card = card;
    ctx->flag = flag;
    ctx->buf = (uint8_t *)bufinfo.buf;
    ctx->block = start_block;
    ctx->nblocks = nblocks;
    ctx->total_blocks = nblocks;
    ctx->is_write = false;
    ctx->abusio_ctx = NULL;

    // Send CMD17 (single) or CMD18 (multi) — synchronous
    int r;
    if (nblocks == 1) {
        r = block_cmd(card, 17, start_block, NULL, 0, false, true);
    } else {
        r = block_cmd(card, 18, start_block, NULL, 0, false, true);
    }
    if (r < 0) {
        extraclock_and_unlock_bus(card);
        mp_raise_OSError(-r);
    }

    // Poll for 0xFE start-block token (synchronous — typically < 1 ms)
    uint8_t token = 0;
    while (token != TOKEN_DATA) {
        common_hal_busio_spi_read(as_busio(card), &token, 1, 0xff);
    }

    // Kick off first DMA transfer for this block
    ctx->abusio_data = make_readinto_data(card, ctx->buf);
    ctx->abusio_ctx = common_hal_abusio_spi_readinto_start(flag, ctx->abusio_data);

    return ctx;
}

mp_obj_t common_hal_asdcardio_asdcard_readblocks_end(void *raw_ctx) {
    asdcardio_transfer_ctx_t *ctx = raw_ctx;
    asdcardio_asdcard_obj_t *card = ctx->card;

    // Collect the DMA result (aborts DMA channels, checks for errors)
    common_hal_abusio_spi_readinto_end(ctx->abusio_ctx);
    ctx->abusio_ctx = NULL;

    // Discard 2-byte CRC-CCITT
    uint8_t crc[2];
    common_hal_busio_spi_read(as_busio(card), crc, 2, 0xff);

    ctx->buf += 512;
    ctx->nblocks--;

    if (ctx->nblocks > 0) {
        // More blocks remain in a CMD18 transfer.
        // Poll for next 0xFE token (synchronous)
        uint8_t token = 0;
        while (token != TOKEN_DATA) {
            common_hal_busio_spi_read(as_busio(card), &token, 1, 0xff);
        }
        // Re-arm flag and start next DMA block
        ctx->block++;
        ctx->abusio_data = make_readinto_data(card, ctx->buf);
        CIRCUITPY_ASYNC_FLAG_INIT(ctx->flag);
        ctx->abusio_ctx = common_hal_abusio_spi_readinto_start(ctx->flag, ctx->abusio_data);
        // Return MP_OBJ_NULL: signals awaitable_iternext() to re-queue and keep polling.
        return MP_OBJ_NULL;
    }

    // Last block done.
    if (ctx->total_blocks > 1) {
        // CMD12 — stop transmission for CMD18
        sd_cmd(card, 12, 0, NULL, 0, true, false);
        uint8_t b;
        uint8_t r = 0;
        while (r != 0) {
            common_hal_busio_spi_read(as_busio(card), &b, 1, 0xff);
            if (b & 0x80) {
                break;
            }
            r = b;
        }
    }

    extraclock_and_unlock_bus(card);
    return mp_const_none;
}

void common_hal_asdcardio_asdcard_readblocks_cancel(void *raw_ctx) {
    asdcardio_transfer_ctx_t *ctx = raw_ctx;
    if (ctx->abusio_ctx) {
        common_hal_abusio_spi_readinto_cancel(ctx->abusio_ctx);
        ctx->abusio_ctx = NULL;
    }
    if (ctx->total_blocks > 1) {
        sd_cmd(ctx->card, 12, 0, NULL, 0, true, false);
    }
    extraclock_and_unlock_bus(ctx->card);
}

// ---- writeblocks async ------------------------------------------------------

void *common_hal_asdcardio_asdcard_writeblocks_start(circuitpy_async_flag_t *flag, mp_obj_t data) {
    mp_obj_t *items;
    size_t len;
    mp_obj_tuple_get(data, &len, &items);
    asdcardio_asdcard_obj_t *card = MP_OBJ_TO_PTR(items[0]);
    uint32_t start_block = (uint32_t)mp_obj_get_int(items[1]);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(items[2], &bufinfo, MP_BUFFER_READ);

    uint32_t nblocks = bufinfo.len / 512;

    if (!lock_and_configure_bus(card)) {
        mp_raise_OSError(EAGAIN);
    }
    uint64_t t_wb_start = common_hal_time_monotonic_ns();

    asdcardio_transfer_ctx_t *ctx = m_new_obj(asdcardio_transfer_ctx_t);
    ctx->card = card;
    ctx->flag = flag;
    ctx->buf = (uint8_t *)bufinfo.buf;
    ctx->block = start_block;
    ctx->nblocks = nblocks;
    ctx->total_blocks = nblocks;
    ctx->is_write = true;
    ctx->abusio_ctx = NULL;

    // Use CMD25 (multi-block write) for nblocks > 1 — the SD card can pipeline
    // sector programming across an erase unit, reducing busy-wait latency vs
    // N × CMD24.  Single-block writes still use CMD24 for simplicity.
    int r;
    uint8_t token;
    if (nblocks > 1) {
        r = block_cmd(card, 25, ctx->block, NULL, 0, false, true);
        token = TOKEN_CMD25;   // 0xFC — CMD25 data-block start token
    } else {
        r = block_cmd(card, 24, ctx->block, NULL, 0, false, true);
        token = TOKEN_DATA;    // 0xFE — CMD24 data token
    }
    if (r < 0) {
        extraclock_and_unlock_bus(card);
        mp_raise_OSError(-r);
    }
    common_hal_busio_spi_write(as_busio(card), &token, 1);

    // Kick off DMA write for 512 bytes
    ctx->phase = ASDCARD_PHASE_DMA;
    ctx->abusio_data = make_write_data(card, ctx->buf);
    ctx->abusio_ctx = common_hal_abusio_spi_write_start(flag, ctx->abusio_data);

    ctx->t_start = t_wb_start;
    ctx->blocking_ns = common_hal_time_monotonic_ns() - t_wb_start;
    ctx->busy_polls = 0;

    return ctx;
}

// Called with bus locked and CS asserted.  Advances ctx to the next block or
// terminates the multi-block transfer.  Returns MP_OBJ_NULL when more work
// remains (DMA or STOP_WAIT), mp_const_none when the transfer is complete.
static mp_obj_t writeblocks_advance(asdcardio_transfer_ctx_t *ctx) {
    asdcardio_asdcard_obj_t *card = ctx->card;
    ctx->buf += 512;
    ctx->block++;
    ctx->nblocks--;

    if (ctx->nblocks > 0) {
        if (ctx->total_blocks > 1) {
            // CMD25 mode: no new command — just send TOKEN_CMD25 for next block.
            uint8_t tok = TOKEN_CMD25;
            common_hal_busio_spi_write(as_busio(card), &tok, 1);
        } else {
            // CMD24 mode: issue a fresh CMD24 for the next block.
            int r = block_cmd(card, 24, ctx->block, NULL, 0, false, true);
            if (r < 0) {
                extraclock_and_unlock_bus(card);
                mp_raise_OSError(-r);
            }
            uint8_t tok = TOKEN_DATA;
            common_hal_busio_spi_write(as_busio(card), &tok, 1);
        }
        ctx->abusio_data = make_write_data(card, ctx->buf);
        CIRCUITPY_ASYNC_FLAG_INIT(ctx->flag);
        ctx->abusio_ctx = common_hal_abusio_spi_write_start(ctx->flag, ctx->abusio_data);
        ctx->phase = ASDCARD_PHASE_DMA;
        return MP_OBJ_NULL;
    }

    // Last block — CMD25 termination: STOP_TRAN + stuff byte + first busy sample.
    if (ctx->total_blocks > 1) {
        uint8_t stop = TOKEN_STOP_TRAN;
        common_hal_busio_spi_write(as_busio(card), &stop, 1);
        uint8_t b;
        common_hal_busio_spi_read(as_busio(card), &b, 1, 0xff); // stuff byte (Nec)
        common_hal_busio_spi_read(as_busio(card), &b, 1, 0xff); // first busy sample
        if (b == 0x00) {
            ctx->busy_deadline = common_hal_time_monotonic_ns() + WRITE_TIMEOUT_NS;
            ctx->phase = ASDCARD_PHASE_STOP_WAIT;
            cs_deassert(card);
            common_hal_busio_spi_unlock(as_busio(card));
            return MP_OBJ_NULL;
        }
    }

    extraclock_and_unlock_bus(card);
    return mp_const_none;
}

mp_obj_t common_hal_asdcardio_asdcard_writeblocks_end(void *raw_ctx) {
    asdcardio_transfer_ctx_t *ctx = raw_ctx;
    asdcardio_asdcard_obj_t *card = ctx->card;
    uint64_t t0 = common_hal_time_monotonic_ns();
    mp_obj_t result;

    // ── STOP_WAIT: CMD25 STOP_TRAN sent; polling until card releases DO ──────
    //
    // The SD SPI spec says CS should remain asserted during busy, but
    // SDHC/SDXC cards re-assert busy correctly after a CS deassert/reassert
    // cycle, which lets us release the bus between polls.
    if (ctx->phase == ASDCARD_PHASE_STOP_WAIT) {
        if (!lock_and_configure_bus(card)) {
            ctx->blocking_ns += common_hal_time_monotonic_ns() - t0;
            return MP_OBJ_NULL; // bus held by another task — yield and retry
        }
        uint8_t b;
        common_hal_busio_spi_read(as_busio(card), &b, 1, 0xff);
        ctx->busy_polls++;
        if (b == 0x00) {
            if (common_hal_time_monotonic_ns() > ctx->busy_deadline) {
                extraclock_and_unlock_bus(card);
                mp_raise_OSError(MP_ETIMEDOUT);
            }
            cs_deassert(card);
            common_hal_busio_spi_unlock(as_busio(card));
            ctx->blocking_ns += common_hal_time_monotonic_ns() - t0;
            return MP_OBJ_NULL;
        }
        extraclock_and_unlock_bus(card);
        uint64_t t_now = common_hal_time_monotonic_ns();
        ctx->blocking_ns += t_now - t0;
        TIMING_PRINT("writeblocks: %lu blk blocking=%luus total=%luus busy_polls=%lu\n",
            (unsigned long)ctx->total_blocks,
            (unsigned long)(ctx->blocking_ns / 1000),
            (unsigned long)((t_now - ctx->t_start) / 1000),
            (unsigned long)ctx->busy_polls);
        return mp_const_none;
    }

    // ── BUSY_WAIT: per-block programming; polling until card releases DO ──────
    if (ctx->phase == ASDCARD_PHASE_BUSY_WAIT) {
        if (!lock_and_configure_bus(card)) {
            ctx->blocking_ns += common_hal_time_monotonic_ns() - t0;
            return MP_OBJ_NULL; // bus held by another task — yield and retry
        }
        uint8_t b;
        common_hal_busio_spi_read(as_busio(card), &b, 1, 0xff);
        ctx->busy_polls++;
        if (b == 0x00) {
            if (common_hal_time_monotonic_ns() > ctx->busy_deadline) {
                extraclock_and_unlock_bus(card);
                mp_raise_OSError(MP_ETIMEDOUT);
            }
            cs_deassert(card);
            common_hal_busio_spi_unlock(as_busio(card));
            ctx->blocking_ns += common_hal_time_monotonic_ns() - t0;
            return MP_OBJ_NULL;
        }
        // Card ready — bus is locked, CS asserted — advance to next block.
        result = writeblocks_advance(ctx);
        uint64_t t_now = common_hal_time_monotonic_ns();
        ctx->blocking_ns += t_now - t0;
        if (result == mp_const_none) {
            TIMING_PRINT("writeblocks: %lu blk blocking=%luus total=%luus busy_polls=%lu\n",
                (unsigned long)ctx->total_blocks,
                (unsigned long)(ctx->blocking_ns / 1000),
                (unsigned long)((t_now - ctx->t_start) / 1000),
                (unsigned long)ctx->busy_polls);
        }
        return result;
    }

    // ── DMA phase: collect result, send CRC, read response token ─────────────
    common_hal_abusio_spi_write_end(ctx->abusio_ctx);
    ctx->abusio_ctx = NULL;

    // 2 dummy CRC bytes
    uint8_t crc[2] = {0xff, 0xff};
    common_hal_busio_spi_write(as_busio(card), crc, 2);

    // Read data response token
    uint8_t resp = 0;
    for (int i = 0; i < CMD_TIMEOUT; i++) {
        common_hal_busio_spi_read(as_busio(card), &resp, 1, 0xff);
        if ((resp & 0x11) == 0x01) {
            break;
        }
    }
    if ((resp & 0x1f) != 0x05) {
        extraclock_and_unlock_bus(card);
        mp_raise_OSError(MP_EIO);
    }

    // Sample first busy byte; deassert CS and yield if card is still programming.
    uint8_t b;
    common_hal_busio_spi_read(as_busio(card), &b, 1, 0xff);
    if (b == 0x00) {
        ctx->busy_deadline = common_hal_time_monotonic_ns() + WRITE_TIMEOUT_NS;
        ctx->phase = ASDCARD_PHASE_BUSY_WAIT;
        cs_deassert(card);
        common_hal_busio_spi_unlock(as_busio(card));
        ctx->blocking_ns += common_hal_time_monotonic_ns() - t0;
        return MP_OBJ_NULL;
    }

    // Card was already ready — advance immediately.
    result = writeblocks_advance(ctx);
    uint64_t t_now = common_hal_time_monotonic_ns();
    ctx->blocking_ns += t_now - t0;
    if (result == mp_const_none) {
        TIMING_PRINT("writeblocks: %lu blk blocking=%luus total=%luus busy_polls=%lu\n",
            (unsigned long)ctx->total_blocks,
            (unsigned long)(ctx->blocking_ns / 1000),
            (unsigned long)((t_now - ctx->t_start) / 1000),
            (unsigned long)ctx->busy_polls);
    }
    return result;
}

void common_hal_asdcardio_asdcard_writeblocks_cancel(void *raw_ctx) {
    asdcardio_transfer_ctx_t *ctx = raw_ctx;
    if (ctx->phase == ASDCARD_PHASE_BUSY_WAIT || ctx->phase == ASDCARD_PHASE_STOP_WAIT) {
        // Bus is already unlocked and CS deasserted — nothing to tear down.
        return;
    }
    if (ctx->abusio_ctx) {
        common_hal_abusio_spi_write_cancel(ctx->abusio_ctx);
        ctx->abusio_ctx = NULL;
    }
    extraclock_and_unlock_bus(ctx->card);
}

#endif // MICROPY_PY_ASYNC_AWAIT
