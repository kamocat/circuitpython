// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 CircuitPython Contributors
//
// SPDX-License-Identifier: MIT

#pragma once

#include "py/obj.h"
#include "rp2350_sdio.h"

// Per-object state for sdioio.SDCard on RP2350.
typedef struct {
    mp_obj_base_t base;

    // PIO/DMA transport state (inline, not a pointer).
    rp2350_sdio_state_t sdio;

    // SD card properties discovered during init.
    uint32_t sectors;       // total number of 512-byte blocks
    int cdv;                // block-address divisor: 1 (SDHC/SDXC) or 512 (v1)
    uint32_t frequency;     // actual clock frequency in Hz (after configure())
    uint8_t width;          // current bus width: 1 or 4
    uint16_t rca;           // relative card address (from CMD3)

    bool deinited;
    bool never_reset_flag;

    // GPIO numbers of the claimed pins (for reset_pin_number on deinit).
    uint8_t clk_pin_no;
    uint8_t cmd_pin_no;
    uint8_t d0_pin_no;   // D1-D3 are d0_pin_no+1..+3
    uint8_t num_data;
} sdioio_sdcard_obj_t;
