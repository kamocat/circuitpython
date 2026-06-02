// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 CircuitPython Contributors
//
// SPDX-License-Identifier: MIT

#include "py/obj.h"
#include "py/runtime.h"

#include "shared-bindings/asdioio/__init__.h"
#include "shared-bindings/asdioio/ASDioCard.h"

//| """Async SDIO SD card block device using PIO and DMA_IRQ_2 (RP2350 only).
//|
//| Provides ``asdioio.ASDioCard``, which mirrors ``sdioio.SDCard`` but uses an
//| async SDIO transport.  ``readblocks()`` and ``writeblocks()`` are async
//| coroutines that yield to the event loop during each 512-byte DMA transfer,
//| allowing other tasks to run concurrently.
//|
//| The CLK pin is derived automatically as ``(data0 - 2) % 32``.  Data pins
//| D1-D3 must also be free and are the three pins following ``data0``.
//|
//| Example::
//|
//|     import asyncio, board, asdioio, asyncfat
//|
//|     async def main():
//|         sd = asdioio.ASDioCard(board.SDIO_COMMAND, board.SDIO_DATA[0])
//|         f  = await asyncfat.async_open(sd, '/README.TXT', 'r')
//|         print(await f.read(256))
//|         await f.close()
//|
//|     asyncio.run(main())
//| """

#if CIRCUITPY_ASDIOIO

static const mp_rom_map_elem_t asdioio_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_asdioio) },
    { MP_ROM_QSTR(MP_QSTR_ASDioCard), MP_ROM_PTR(&asdioio_ASDioCard_type) },
};
static MP_DEFINE_CONST_DICT(asdioio_module_globals, asdioio_module_globals_table);

const mp_obj_module_t asdioio_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&asdioio_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_asdioio, asdioio_module);

#endif // CIRCUITPY_ASDIOIO
