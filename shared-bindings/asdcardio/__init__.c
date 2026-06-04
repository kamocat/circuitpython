// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2026 CircuitPython Contributors
//
// SPDX-License-Identifier: MIT

#include "py/obj.h"
#include "py/runtime.h"

#include "shared-bindings/asdcardio/__init__.h"
#include "shared-bindings/asdcardio/ASdCard.h"
#include "shared-bindings/asdcardio/WriteStream.h"

//| """Async SD card block device using DMA-backed SPI transfers.
//|
//| Provides ``asdcardio.ASdCard``, which mirrors ``sdcardio.SDCard`` but takes
//| an ``abusio.SPI`` instance.  ``readblocks()`` and ``writeblocks()`` are async
//| coroutines that yield to the event loop during each 512-byte DMA transfer.
//|
//| Example::
//|
//|     import asyncio, board, abusio, digitalio, asdcardio, asyncfat
//|
//|     async def main():
//|         spi = abusio.SPI(board.GP18, board.GP19, board.GP16)
//|         cs  = digitalio.DigitalInOut(board.GP17)
//|         sd  = asdcardio.ASdCard(spi, cs)
//|         f   = await asyncfat.async_open(sd, '/README.TXT', 'r')
//|         print(await f.read(256))
//|         await f.close()
//|
//|     asyncio.run(main())
//| """

static const mp_rom_map_elem_t asdcardio_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_asdcardio) },
    { MP_ROM_QSTR(MP_QSTR_ASdCard),     MP_ROM_PTR(&asdcardio_ASdCard_type) },
    #if MICROPY_PY_ASYNC_AWAIT
    { MP_ROM_QSTR(MP_QSTR_WriteStream), MP_ROM_PTR(&asdcardio_WriteStream_type) },
    #endif
};
static MP_DEFINE_CONST_DICT(asdcardio_module_globals, asdcardio_module_globals_table);

const mp_obj_module_t asdcardio_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&asdcardio_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_asdcardio, asdcardio_module);
