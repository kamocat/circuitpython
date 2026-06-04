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

#include "shared-bindings/asdcardio/WriteStream.h"

#if MICROPY_PY_ASYNC_AWAIT

// ---- helpers ----------------------------------------------------------------

static asdcardio_write_stream_obj_t *check_stream(mp_obj_t self_in) {
    asdcardio_write_stream_obj_t *self = mp_arg_validate_type(
        self_in, &asdcardio_WriteStream_type, MP_QSTR_self);
    if (!self->is_open) {
        mp_raise_ValueError(MP_ERROR_TEXT("WriteStream is closed"));
    }
    return self;
}

// ---- async write ------------------------------------------------------------

//| class WriteStream:
//|     """An open CMD25 write session on an SD card.
//|
//|     Keeps the SPI bus locked across consecutive ``write()`` calls,
//|     eliminating the CMD25 setup overhead between writes.  During per-block
//|     programming (BUSY_WAIT) the bus is released so other SPI tasks can run.
//|
//|     Obtain via :meth:`ASdCard.open_write_stream`.  Must be closed with
//|     :meth:`close` or used as an async context manager (``async with``).
//|     """
//|
//|     async def write(self, buf: ReadableBuffer) -> None:
//|         """Write one or more 512-byte blocks continuing the CMD25 session.
//|
//|         The buffer length must be a non-zero multiple of 512.
//|         The stream's internal block address advances by ``len(buf) // 512``.
//|
//|         :param ~circuitpython_typing.ReadableBuffer buf: Source data.
//|         """
//|         ...
//|
static mp_obj_t asdcardio_write_stream_py_write(mp_obj_t self_in, mp_obj_t buf_in) {
    asdcardio_write_stream_obj_t *self = check_stream(self_in);
    (void)self;  // validated; self_in carried through the tuple

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_READ);
    if (bufinfo.len == 0 || bufinfo.len % 512 != 0) {
        mp_raise_ValueError_varg(
            MP_ERROR_TEXT("Buffer must be a multiple of %d bytes"), 512);
    }

    mp_obj_t mv = mp_obj_new_memoryview('B', bufinfo.len, bufinfo.buf);
    mp_obj_t items[2] = { self_in, mv };
    mp_obj_t data = mp_obj_new_tuple(2, items);

    return circuitpy_awaitable_new(
        common_hal_asdcardio_write_stream_write_start,
        common_hal_asdcardio_write_stream_write_end,
        common_hal_asdcardio_write_stream_write_cancel,
        data);
}
static MP_DEFINE_CONST_FUN_OBJ_2(asdcardio_write_stream_write_obj,
    asdcardio_write_stream_py_write);

// ---- async close ------------------------------------------------------------

//|     async def close(self) -> None:
//|         """Send STOP_TRAN, wait for the card to finish, and release the bus.
//|
//|         Idempotent — calling ``close()`` on an already-closed stream is a
//|         no-op.
//|         """
//|         ...
//|
static mp_obj_t asdcardio_write_stream_py_close(mp_obj_t self_in) {
    // Accept a closed stream — idempotent.
    mp_arg_validate_type(self_in, &asdcardio_WriteStream_type, MP_QSTR_self);

    return circuitpy_awaitable_new(
        common_hal_asdcardio_write_stream_close_start,
        common_hal_asdcardio_write_stream_close_end,
        common_hal_asdcardio_write_stream_close_cancel,
        self_in);
}
static MP_DEFINE_CONST_FUN_OBJ_1(asdcardio_write_stream_close_obj,
    asdcardio_write_stream_py_close);

// ---- async context manager --------------------------------------------------

//|     async def __aenter__(self) -> WriteStream:
//|         """Return self."""
//|         ...
//|
//|     async def __aexit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> bool:
//|         """Close the stream, suppressing no exceptions."""
//|         ...
//|
static mp_obj_t asdcardio_write_stream_aenter(mp_obj_t self_in) {
    mp_arg_validate_type(self_in, &asdcardio_WriteStream_type, MP_QSTR_self);
    return self_in;
}
static MP_DEFINE_CONST_FUN_OBJ_1(asdcardio_write_stream_aenter_obj,
    asdcardio_write_stream_aenter);

static mp_obj_t asdcardio_write_stream_aexit(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    // args[0]=self, args[1]=exc_type, args[2]=exc_val, args[3]=exc_tb
    return asdcardio_write_stream_py_close(args[0]);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(asdcardio_write_stream_aexit_obj, 4, 4,
    asdcardio_write_stream_aexit);

// ---- type definition --------------------------------------------------------

static const mp_rom_map_elem_t asdcardio_write_stream_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_write),      MP_ROM_PTR(&asdcardio_write_stream_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_close),      MP_ROM_PTR(&asdcardio_write_stream_close_obj) },
    { MP_ROM_QSTR(MP_QSTR___aenter__), MP_ROM_PTR(&asdcardio_write_stream_aenter_obj) },
    { MP_ROM_QSTR(MP_QSTR___aexit__),  MP_ROM_PTR(&asdcardio_write_stream_aexit_obj) },
};
static MP_DEFINE_CONST_DICT(asdcardio_write_stream_locals_dict,
    asdcardio_write_stream_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    asdcardio_WriteStream_type,
    MP_QSTR_WriteStream,
    MP_TYPE_FLAG_NONE,
    locals_dict, &asdcardio_write_stream_locals_dict
    );

#endif // MICROPY_PY_ASYNC_AWAIT
