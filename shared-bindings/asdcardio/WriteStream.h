// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2026 CircuitPython Contributors
//
// SPDX-License-Identifier: MIT

#pragma once

#include "py/obj.h"
#include "shared-module/asdcardio/ASdCard.h"

#if MICROPY_PY_ASYNC_AWAIT
extern const mp_obj_type_t asdcardio_WriteStream_type;
#endif
