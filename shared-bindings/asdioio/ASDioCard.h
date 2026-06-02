// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 CircuitPython Contributors
//
// SPDX-License-Identifier: MIT

#pragma once

#include "py/obj.h"
#include "py/mperrno.h"

#include "common-hal/microcontroller/Pin.h"
#include "common-hal/asdioio/ASDioCard.h"

#if MICROPY_PY_ASYNC_AWAIT

// Type object used in Python.  Shared between ports.
extern const mp_obj_type_t asdioio_ASDioCard_type;

#endif // MICROPY_PY_ASYNC_AWAIT
