// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2026 CircuitPython Contributors
//
// SPDX-License-Identifier: MIT

#include "py/obj.h"
#include "py/objarray.h"
#include "py/objtuple.h"
#include "py/runtime.h"
#include "py/circuitpy_objawaitable.h"

#include "shared-bindings/asdcardio/ASdCard.h"
#include "shared-bindings/abusio/SPI.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "shared-bindings/util.h"
#include "shared/runtime/buffer_helper.h"

#if MICROPY_PY_ASYNC_AWAIT

// ---- helpers ----------------------------------------------------------------

static asdcardio_asdcard_obj_t *check_self(mp_obj_t self_in) {
    asdcardio_asdcard_obj_t *self = mp_arg_validate_type(
        self_in, &asdcardio_ASdCard_type, MP_QSTR_self);
    if (common_hal_asdcardio_asdcard_deinited(self)) {
        raise_deinited_error();
    }
    return self;
}

// ---- constructor ------------------------------------------------------------

//| class ASdCard:
//|     """Async SD Card Block Interface
//|
//|     Controls an SD card over an async SPI bus (`abusio.SPI`).
//|     `readblocks()` and `writeblocks()` are async and yield to the event loop
//|     during each 512-byte DMA data phase, allowing other tasks to run.
//|
//|     Usually used with ``asyncfat.async_open()`` for full async file I/O.
//|
//|     Example::
//|
//|         import asyncio, board, abusio, digitalio, asdcardio, asyncfat
//|
//|         async def main():
//|             spi = abusio.SPI(board.GP18, board.GP19, board.GP16)
//|             cs  = digitalio.DigitalInOut(board.GP17)
//|             sd  = asdcardio.ASdCard(spi, cs)
//|             f   = await asyncfat.async_open(sd, '/README.TXT', 'r')
//|             print(await f.read(256))
//|             await f.close()
//|
//|         asyncio.run(main())
//|     """
//|
//|     def __init__(
//|         self,
//|         spi: abusio.SPI,
//|         cs: microcontroller.Pin,
//|         baudrate: int = 8000000,
//|     ) -> None:
//|         """Construct an async SD card object.
//|
//|         :param abusio.SPI spi: The async SPI bus.
//|         :param microcontroller.Pin cs: Chip-select pin connected to the card.
//|         :param int baudrate: SPI clock rate used after card initialisation.
//|
//|         During detection and setup a fixed slow rate (250 kHz) is used.
//|         The bus is locked by this object for the duration of every transfer.
//|         """
//|         ...
//|
static mp_obj_t asdcardio_asdcard_make_new(const mp_obj_type_t *type,
    size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_spi, ARG_cs, ARG_baudrate, NUM_ARGS };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_spi,      MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_cs,       MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_baudrate, MP_ARG_INT, {.u_int = 8000000} },
    };
    MP_STATIC_ASSERT(MP_ARRAY_SIZE(allowed_args) == NUM_ARGS);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    abusio_spi_obj_t *spi = mp_arg_validate_type(
        args[ARG_spi].u_obj, &abusio_spi_type, MP_QSTR_spi);
    const mcu_pin_obj_t *cs = validate_obj_is_free_pin(args[ARG_cs].u_obj, MP_QSTR_cs);

    asdcardio_asdcard_obj_t *self = mp_obj_malloc_with_finaliser(
        asdcardio_asdcard_obj_t, &asdcardio_ASdCard_type);

    common_hal_asdcardio_asdcard_construct(self, spi, cs, args[ARG_baudrate].u_int);
    return MP_OBJ_FROM_PTR(self);
}

// ---- count ------------------------------------------------------------------

//|     def count(self) -> int:
//|         """Return the total number of 512-byte sectors on the card."""
//|         ...
//|
static mp_obj_t asdcardio_asdcard_count(mp_obj_t self_in) {
    asdcardio_asdcard_obj_t *self = check_self(self_in);
    return mp_obj_new_int_from_ull(common_hal_asdcardio_asdcard_get_blockcount(self));
}
static MP_DEFINE_CONST_FUN_OBJ_1(asdcardio_asdcard_count_obj, asdcardio_asdcard_count);

// ---- deinit -----------------------------------------------------------------

//|     def deinit(self) -> None:
//|         """Release the CS pin and mark the object as deinitialised."""
//|         ...
//|
static mp_obj_t asdcardio_asdcard_deinit(mp_obj_t self_in) {
    asdcardio_asdcard_obj_t *self = mp_arg_validate_type(
        self_in, &asdcardio_ASdCard_type, MP_QSTR_self);
    common_hal_asdcardio_asdcard_deinit(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(asdcardio_asdcard_deinit_obj, asdcardio_asdcard_deinit);

// ---- sync -------------------------------------------------------------------

//|     def sync(self) -> None:
//|         """Ensure all written blocks are committed to the card."""
//|         ...
//|
static mp_obj_t asdcardio_asdcard_sync(mp_obj_t self_in) {
    asdcardio_asdcard_obj_t *self = check_self(self_in);
    int r = common_hal_asdcardio_asdcard_sync(self);
    if (r < 0) {
        mp_raise_OSError(-r);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(asdcardio_asdcard_sync_obj, asdcardio_asdcard_sync);

// ---- async readblocks -------------------------------------------------------

//|     async def readblocks(self, start_block: int, buf: WriteableBuffer) -> None:
//|         """Read one or more 512-byte blocks from the card asynchronously.
//|
//|         Yields to the asyncio event loop during each DMA data transfer.
//|
//|         :param int start_block: First block to read.
//|         :param ~circuitpython_typing.WriteableBuffer buf: Destination buffer.
//|             Length must be a multiple of 512.
//|         """
//|         ...
//|
static mp_obj_t asdcardio_sdcard_py_readblocks(mp_obj_t self_in,
    mp_obj_t start_block_in, mp_obj_t buf_in) {
    (void)check_self(self_in);  // deinit check only; self used via self_in in tuple

    uint32_t start_block = (uint32_t)mp_obj_get_int(start_block_in);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_WRITE);
    if (bufinfo.len == 0 || bufinfo.len % 512 != 0) {
        mp_raise_ValueError_varg(
            MP_ERROR_TEXT("Buffer must be a multiple of %d bytes"), 512);
    }

    // Pack (self, start_block, buf_memoryview) as a 3-tuple passed through
    // the awaitable data field to readblocks_start().
    mp_obj_t mv = mp_obj_new_memoryview(
        'B' | MP_OBJ_ARRAY_TYPECODE_FLAG_RW, bufinfo.len, bufinfo.buf);
    mp_obj_t items[3] = {
        self_in,
        mp_obj_new_int_from_ull(start_block),
        mv,
    };
    mp_obj_t data = mp_obj_new_tuple(3, items);

    return circuitpy_awaitable_new(
        common_hal_asdcardio_asdcard_readblocks_start,
        common_hal_asdcardio_asdcard_readblocks_end,
        common_hal_asdcardio_asdcard_readblocks_cancel,
        data);
}
static MP_DEFINE_CONST_FUN_OBJ_3(asdcardio_asdcard_readblocks_obj,
    asdcardio_sdcard_py_readblocks);

// ---- async writeblocks ------------------------------------------------------

//|     async def writeblocks(self, start_block: int, buf: ReadableBuffer) -> None:
//|         """Write one or more 512-byte blocks to the card asynchronously.
//|
//|         Yields to the asyncio event loop during each DMA data transfer.
//|
//|         :param int start_block: First block to write.
//|         :param ~circuitpython_typing.ReadableBuffer buf: Source buffer.
//|             Length must be a multiple of 512.
//|         """
//|         ...
//|
static mp_obj_t asdcardio_sdcard_py_writeblocks(mp_obj_t self_in,
    mp_obj_t start_block_in, mp_obj_t buf_in) {
    (void)check_self(self_in);  // deinit check only; self used via self_in in tuple

    uint32_t start_block = (uint32_t)mp_obj_get_int(start_block_in);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_READ);
    if (bufinfo.len == 0 || bufinfo.len % 512 != 0) {
        mp_raise_ValueError_varg(
            MP_ERROR_TEXT("Buffer must be a multiple of %d bytes"), 512);
    }

    mp_obj_t mv = mp_obj_new_memoryview('B', bufinfo.len, bufinfo.buf);
    mp_obj_t items[3] = {
        self_in,
        mp_obj_new_int_from_ull(start_block),
        mv,
    };
    mp_obj_t data = mp_obj_new_tuple(3, items);

    return circuitpy_awaitable_new(
        common_hal_asdcardio_asdcard_writeblocks_start,
        common_hal_asdcardio_asdcard_writeblocks_end,
        common_hal_asdcardio_asdcard_writeblocks_cancel,
        data);
}
static MP_DEFINE_CONST_FUN_OBJ_3(asdcardio_asdcard_writeblocks_obj,
    asdcardio_sdcard_py_writeblocks);

// ---- type definition --------------------------------------------------------

static const mp_rom_map_elem_t asdcardio_asdcard_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_count),       MP_ROM_PTR(&asdcardio_asdcard_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),      MP_ROM_PTR(&asdcardio_asdcard_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR___del__),     MP_ROM_PTR(&asdcardio_asdcard_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_sync),        MP_ROM_PTR(&asdcardio_asdcard_sync_obj) },
    { MP_ROM_QSTR(MP_QSTR_readblocks),  MP_ROM_PTR(&asdcardio_asdcard_readblocks_obj) },
    { MP_ROM_QSTR(MP_QSTR_writeblocks), MP_ROM_PTR(&asdcardio_asdcard_writeblocks_obj) },
};
static MP_DEFINE_CONST_DICT(asdcardio_asdcard_locals_dict,
    asdcardio_asdcard_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    asdcardio_ASdCard_type,
    MP_QSTR_ASdCard,
    MP_TYPE_FLAG_NONE,
    make_new, asdcardio_asdcard_make_new,
    locals_dict, &asdcardio_asdcard_locals_dict
    );

#endif // MICROPY_PY_ASYNC_AWAIT
