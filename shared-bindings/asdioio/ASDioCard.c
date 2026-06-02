// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 CircuitPython Contributors
//
// SPDX-License-Identifier: MIT

// Python bindings for asdioio.ASDioCard.
// Async SDIO SD card block device using RP2350 PIO and DMA_IRQ_2.

#include "py/obj.h"
#include "py/objarray.h"
#include "py/objtuple.h"
#include "py/runtime.h"
#include "py/circuitpy_objawaitable.h"

#include "shared-bindings/asdioio/ASDioCard.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "shared-bindings/util.h"
#include "shared/runtime/buffer_helper.h"

#if MICROPY_PY_ASYNC_AWAIT

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static asdioio_asdiocard_obj_t *check_self(mp_obj_t self_in) {
    asdioio_asdiocard_obj_t *self = mp_arg_validate_type(
        self_in, &asdioio_ASDioCard_type, MP_QSTR_self);
    if (common_hal_asdioio_asdiocard_deinited(self)) {
        raise_deinited_error();
    }
    return self;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

//| class ASDioCard:
//|     """Async SDIO SD Card Block Interface (RP2350 only)
//|
//|     Controls an SD card over the SDIO bus using PIO state machines.
//|     ``readblocks()`` and ``writeblocks()`` are async coroutines that yield
//|     to the asyncio event loop during each 512-byte DMA transfer.
//|
//|     The CLK pin is derived automatically as ``(data0 - 2) % 32``.
//|     Data pins D1-D3 must be the three GPIO pins directly above ``data0``
//|     and must all be free.
//|
//|     Usually used with ``asyncfat.async_open()`` for full async file I/O.
//|
//|     Example::
//|
//|         import asyncio, board, asdioio, asyncfat
//|
//|         async def main():
//|             sd = asdioio.ASDioCard(board.SDIO_COMMAND, board.SDIO_DATA[0])
//|             f  = await asyncfat.async_open(sd, '/README.TXT', 'r')
//|             print(await f.read(256))
//|             await f.close()
//|
//|         asyncio.run(main())
//|     """
//|
//|     def __init__(
//|         self,
//|         command: microcontroller.Pin,
//|         data0: microcontroller.Pin,
//|         frequency: int = 25000000,
//|     ) -> None:
//|         """Construct an async SDIO SD card object.
//|
//|         :param ~microcontroller.Pin command: CMD pin.
//|         :param ~microcontroller.Pin data0: D0 pin (D1-D3 and CLK derived from it).
//|         :param int frequency: Target SDIO clock in Hz. Defaults to 25 MHz.
//|
//|         During card detection a 400 kHz clock is used; the requested
//|         frequency takes effect after successful initialisation.
//|         """
//|         ...
//|
static mp_obj_t asdioio_asdiocard_make_new(const mp_obj_type_t *type,
    size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_command, ARG_data0, ARG_frequency, NUM_ARGS };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_command,   MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_data0,     MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_frequency, MP_ARG_INT, {.u_int = 25000000} },
    };
    MP_STATIC_ASSERT(MP_ARRAY_SIZE(allowed_args) == NUM_ARGS);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args,
        MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    const mcu_pin_obj_t *command = validate_obj_is_free_pin(
        args[ARG_command].u_obj, MP_QSTR_command);
    const mcu_pin_obj_t *data0 = validate_obj_is_free_pin(
        args[ARG_data0].u_obj, MP_QSTR_data0);

    asdioio_asdiocard_obj_t *self = mp_obj_malloc_with_finaliser(
        asdioio_asdiocard_obj_t, &asdioio_ASDioCard_type);

    common_hal_asdioio_asdiocard_construct(self, command, data0,
        (uint32_t)args[ARG_frequency].u_int);
    return MP_OBJ_FROM_PTR(self);
}

// ---------------------------------------------------------------------------
// count
// ---------------------------------------------------------------------------

//|     def count(self) -> int:
//|         """Return the total number of 512-byte sectors on the card."""
//|         ...
//|
static mp_obj_t asdioio_asdiocard_count(mp_obj_t self_in) {
    asdioio_asdiocard_obj_t *self = check_self(self_in);
    return mp_obj_new_int_from_ull(
        common_hal_asdioio_asdiocard_get_count(self));
}
static MP_DEFINE_CONST_FUN_OBJ_1(asdioio_asdiocard_count_obj, asdioio_asdiocard_count);

// ---------------------------------------------------------------------------
// deinit / __del__
// ---------------------------------------------------------------------------

//|     def deinit(self) -> None:
//|         """Permanently release PIO and DMA resources."""
//|         ...
//|
static mp_obj_t asdioio_asdiocard_deinit(mp_obj_t self_in) {
    asdioio_asdiocard_obj_t *self = mp_arg_validate_type(
        self_in, &asdioio_ASDioCard_type, MP_QSTR_self);
    common_hal_asdioio_asdiocard_deinit(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(asdioio_asdiocard_deinit_obj, asdioio_asdiocard_deinit);

// ---------------------------------------------------------------------------
// async readblocks
// ---------------------------------------------------------------------------

//|     async def readblocks(self, start_block: int, buf: WriteableBuffer) -> None:
//|         """Read one or more 512-byte blocks asynchronously.
//|
//|         Yields to the asyncio event loop during each DMA transfer.
//|
//|         :param int start_block: First block index to read.
//|         :param ~circuitpython_typing.WriteableBuffer buf: Destination.
//|             Length must be a non-zero multiple of 512.
//|         """
//|         ...
//|
static mp_obj_t asdioio_asdiocard_py_readblocks(mp_obj_t self_in,
    mp_obj_t start_block_in, mp_obj_t buf_in) {
    (void)check_self(self_in); // deinit check; actual self accessed inside start()

    uint32_t start_block = (uint32_t)mp_obj_get_int(start_block_in);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_WRITE);
    if (bufinfo.len == 0 || bufinfo.len % 512 != 0) {
        mp_raise_ValueError_varg(
            MP_ERROR_TEXT("Buffer must be a multiple of %d bytes"), 512);
    }

    // Pass (self, start_block, buf) to start() via a 3-tuple.
    mp_obj_t mv = mp_obj_new_memoryview(
        'B' | MP_OBJ_ARRAY_TYPECODE_FLAG_RW, bufinfo.len, bufinfo.buf);
    mp_obj_t items[3] = {
        self_in,
        mp_obj_new_int_from_ull(start_block),
        mv,
    };
    mp_obj_t data = mp_obj_new_tuple(3, items);

    return circuitpy_awaitable_new(
        common_hal_asdioio_asdiocard_readblocks_start,
        common_hal_asdioio_asdiocard_readblocks_end,
        common_hal_asdioio_asdiocard_readblocks_cancel,
        data);
}
static MP_DEFINE_CONST_FUN_OBJ_3(asdioio_asdiocard_readblocks_obj,
    asdioio_asdiocard_py_readblocks);

// ---------------------------------------------------------------------------
// async writeblocks
// ---------------------------------------------------------------------------

//|     async def writeblocks(self, start_block: int, buf: ReadableBuffer) -> None:
//|         """Write one or more 512-byte blocks asynchronously.
//|
//|         Yields to the asyncio event loop during each DMA transfer.
//|
//|         :param int start_block: First block index to write.
//|         :param ~circuitpython_typing.ReadableBuffer buf: Source buffer.
//|             Length must be a non-zero multiple of 512.
//|         """
//|         ...
//|
static mp_obj_t asdioio_asdiocard_py_writeblocks(mp_obj_t self_in,
    mp_obj_t start_block_in, mp_obj_t buf_in) {
    (void)check_self(self_in);

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
        common_hal_asdioio_asdiocard_writeblocks_start,
        common_hal_asdioio_asdiocard_writeblocks_end,
        common_hal_asdioio_asdiocard_writeblocks_cancel,
        data);
}
static MP_DEFINE_CONST_FUN_OBJ_3(asdioio_asdiocard_writeblocks_obj,
    asdioio_asdiocard_py_writeblocks);

// ---------------------------------------------------------------------------
// Type definition
// ---------------------------------------------------------------------------

static const mp_rom_map_elem_t asdioio_asdiocard_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_count),       MP_ROM_PTR(&asdioio_asdiocard_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),      MP_ROM_PTR(&asdioio_asdiocard_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR___del__),     MP_ROM_PTR(&asdioio_asdiocard_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_readblocks),  MP_ROM_PTR(&asdioio_asdiocard_readblocks_obj) },
    { MP_ROM_QSTR(MP_QSTR_writeblocks), MP_ROM_PTR(&asdioio_asdiocard_writeblocks_obj) },
};
static MP_DEFINE_CONST_DICT(asdioio_asdiocard_locals_dict,
    asdioio_asdiocard_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    asdioio_ASDioCard_type,
    MP_QSTR_ASDioCard,
    MP_TYPE_FLAG_NONE,
    make_new, asdioio_asdiocard_make_new,
    locals_dict, &asdioio_asdiocard_locals_dict
    );

#endif // MICROPY_PY_ASYNC_AWAIT
