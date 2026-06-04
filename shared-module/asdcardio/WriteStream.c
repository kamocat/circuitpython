// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2026 CircuitPython Contributors
//
// SPDX-License-Identifier: MIT

// WriteStream has no shared-module implementation — it lives entirely in
// shared-bindings/asdcardio/WriteStream.c and
// shared-module/asdcardio/ASdCard.c (the common_hal_asdcardio_write_stream_*
// functions).  This stub satisfies the build system's requirement that every
// SRC_SHARED_MODULE entry has a file under both shared-bindings/ and
// shared-module/.
