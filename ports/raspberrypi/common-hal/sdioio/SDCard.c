// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 CircuitPython Contributors
//
// SPDX-License-Identifier: MIT
//
// RP2350 implementation of sdioio.SDCard using PIO + DMA SDIO.

#include <string.h>
#include "py/mperrno.h"
#include "py/runtime.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "shared-bindings/sdioio/SDCard.h"
#include "shared-bindings/time/__init__.h"
#include "supervisor/shared/translate/translate.h"

#include "SDCard.h"
#include "rp2350_sdio.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "extmod/vfs.h"
#include "common-hal/microcontroller/Pin.h"

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

static inline uint64_t now_ms(void) {
    return common_hal_time_monotonic_ms();
}

// ---------------------------------------------------------------------------
// SD card commands and bit patterns
// ---------------------------------------------------------------------------

#define CMD0    0   // GO_IDLE_STATE
#define CMD2    2   // ALL_SEND_CID
#define CMD3    3   // SEND_RELATIVE_ADDR
#define CMD6    6   // SWITCH_FUNC
#define CMD7    7   // SELECT_CARD
#define CMD8    8   // SEND_IF_COND
#define CMD12   12  // STOP_TRANSMISSION
#define CMD16   16  // SET_BLOCKLEN
#define CMD17   17  // READ_SINGLE_BLOCK
#define CMD18   18  // READ_MULTIPLE_BLOCK
#define CMD24   24  // WRITE_BLOCK
#define CMD25   25  // WRITE_MULTIPLE_BLOCK
#define CMD55   55  // APP_CMD
#define CMD58   58  // READ_OCR (SPI mode only)
#define ACMD6   6   // SET_BUS_WIDTH (issued after CMD55)
#define ACMD41  41  // SD_SEND_OP_COND (issued after CMD55)
#define ACMD42  42  // SET_CLR_CARD_DETECT (issued after CMD55)

#define R1_IDLE_STATE       0x01u
#define R1_ILLEGAL_CMD      0x04u

#define OCR_BUSY            0x80000000u
#define OCR_SDHC            0x40000000u
#define OCR_VDD_33          0x003C0000u   // 3.0–3.4 V window

// ---------------------------------------------------------------------------
// Helper: send ACMD (CMD55 prefix + command)
// ---------------------------------------------------------------------------

static sdio_status_t send_acmd(sdioio_sdcard_obj_t *self,
    uint8_t cmd, uint32_t arg, uint32_t *response) {
    uint32_t r1 = 0;
    sdio_status_t st = rp2350_sdio_command_R1(&self->sdio, CMD55,
        (uint32_t)self->rca << 16, &r1);
    if (st != SDIO_OK) {
        return st;
    }
    // After card init, RCA is 0 for CMD55; ignore R1 value during init.
    // ACMD41 returns R3 (no real CRC) — use R3 variant to skip CRC check.
    if (cmd == ACMD41) {
        return rp2350_sdio_command_R3(&self->sdio, cmd, arg, response);
    }
    return rp2350_sdio_command_R1(&self->sdio, cmd, arg, response);
}

// ---------------------------------------------------------------------------
// Card initialisation sequence
// ---------------------------------------------------------------------------

// Set to 1 to print diagnostic messages on init failures.
#if 1
#define SDIO_INIT_DEBUG(...) mp_printf(&mp_plat_print, __VA_ARGS__)
#else
#define SDIO_INIT_DEBUG(...) (void)0
#endif

static const char *sdio_status_name(sdio_status_t st) {
    switch (st) {
        case SDIO_OK:
            return "OK";
        case SDIO_ERR_RESPONSE_TIMEOUT:
            return "RESPONSE_TIMEOUT";
        case SDIO_ERR_RESPONSE_CRC:
            return "RESPONSE_CRC";
        case SDIO_ERR_RESPONSE_CODE:
            return "RESPONSE_CODE";
        case SDIO_ERR_DATA_TIMEOUT:
            return "DATA_TIMEOUT";
        case SDIO_ERR_DATA_CRC:
            return "DATA_CRC";
        case SDIO_ERR_WRITE_CRC:
            return "WRITE_CRC";
        case SDIO_ERR_WRITE_FAIL:
            return "WRITE_FAIL";
        case SDIO_ERR_INIT:
            return "INIT";
        default:
            return "UNKNOWN";
    }
}

static int sdioio_init_card(sdioio_sdcard_obj_t *self) {
    sdio_status_t st;
    uint32_t resp = 0;

    SDIO_INIT_DEBUG("sdioio: init_card start: clk=%u cmd=%u d0=%u width=%u\n",
        self->clk_pin_no, self->cmd_pin_no, self->d0_pin_no, self->width);

    // -------------------------------------------------------------------------
    // SPI-mode escape: if the card was left in SPI mode by a prior session
    // (e.g. via busio/asdcardio or a reset mid-write), it will ignore all
    // native SDIO commands.  The only software escape is to send CMD0 with
    // CS (D3) asserted LOW while the card is in SPI mode, which resets it to
    // Idle state — still in SPI mode, but at least responsive.  After that,
    // deasserting CS and cycling the native init sequence should bring it back
    // to native mode... but the SD spec actually says: once in SPI mode, only
    // a VCC power cycle exits it.
    //
    // What we CAN do is detect SPI mode early: temporarily take D3 as a GPIO
    // output, drive it LOW, send a bitbang CMD0 (SPI framing: 0x40,0,0,0,0,0x95),
    // release D3, and proceed.  If the card was in SPI mode, CMD0 resets it
    // (card stays in SPI mode but is now Idle); if the card was in native mode,
    // CMD0 in SPI framing is illegal but harmless — the card will NAK it.
    // Either way, after this block the CLK/CMD pins go back to PIO control
    // and the native init proceeds.  On a true power cycle the native init
    // will succeed; if the card truly cannot exit SPI mode the caller will
    // get ETIMEDOUT and the user must unplug USB power.
    {
        // Disable the CMD SM temporarily so bitbang can drive CLK and CMD.
        pio_sm_set_enabled(self->sdio.pio, self->sdio.cmd_sm, false);

        uint clk = self->clk_pin_no;
        uint cmd = self->cmd_pin_no;
        uint cs = self->d0_pin_no + 3;  // D3 = CS in SPI mode

        // Reconfigure CLK and CMD as SIO (software-controlled GPIO) outputs.
        gpio_init(clk);
        gpio_set_dir(clk, true);
        gpio_put(clk, 0);

        gpio_init(cmd);
        gpio_set_dir(cmd, true);
        gpio_put(cmd, 1); // MOSI idle high

        // CS as GPIO output, initially HIGH.
        gpio_init(cs);
        gpio_set_dir(cs, true);
        gpio_put(cs, 1);

        // Send >= 74 clocks with CS=HIGH and MOSI=HIGH (SD spec power-on init).
        for (int i = 0; i < 80; i++) {
            gpio_put(clk, 0);
            gpio_put(clk, 1);
        }
        gpio_put(clk, 0);

        // Assert CS (SPI mode).
        gpio_put(cs, 0);

        // Bitbang CMD0 in SPI framing: {0x40, 0x00, 0x00, 0x00, 0x00, 0x95}
        // (start=0, tx=1, idx=0, arg=0, CRC7=0x4A | end-bit=1).
        static const uint8_t cmd0_spi[6] = {0x40, 0x00, 0x00, 0x00, 0x00, 0x95};
        for (int b = 0; b < 6; b++) {
            for (int bit = 7; bit >= 0; bit--) {
                gpio_put(cmd, (cmd0_spi[b] >> bit) & 1);
                gpio_put(clk, 0);
                gpio_put(clk, 1);
            }
        }
        // Clock out 8 more bits (CMD/MOSI high) to let card process the command.
        gpio_put(cmd, 1);
        for (int i = 0; i < 8; i++) {
            gpio_put(clk, 0);
            gpio_put(clk, 1);
        }
        gpio_put(clk, 0);

        // Deassert CS.
        gpio_put(cs, 1);

        // Send 8 more clocks with CS=HIGH to complete any pending SPI response.
        for (int i = 0; i < 8; i++) {
            gpio_put(clk, 0);
            gpio_put(clk, 1);
        }
        gpio_put(clk, 0);

        SDIO_INIT_DEBUG("sdioio: SPI-mode escape CMD0 sent\n");

        // Return CLK, CMD, CS/D3 back to PIO function.
        gpio_function_t fn = (self->sdio.pio == pio1) ? GPIO_FUNC_PIO1 : GPIO_FUNC_PIO0;
        gpio_set_function(clk, fn);
        gpio_pull_up(clk);  // RP2350 pull-up on CLK keeps line high when SM idles

        gpio_set_function(cmd, fn);
        gpio_pull_up(cmd);

        // D3/CS: return to PIO input with pull-up.
        gpio_set_function(cs, fn);
        gpio_pull_up(cs);

        // Re-enable the CMD SM.  It restarts from the beginning of the program.
        pio_sm_restart(self->sdio.pio, self->sdio.cmd_sm);
        pio_sm_exec(self->sdio.pio, self->sdio.cmd_sm,
            pio_encode_jmp(self->sdio.pio_cmd_clk_offset));
        pio_sm_set_enabled(self->sdio.pio, self->sdio.cmd_sm, true);

        // Brief settle time.
        common_hal_time_delay_ms(2);
    }

    // Give the card >= 74 CLK cycles (SD spec minimum) with CMD high so it can
    // finish any pending operation from a previous session (e.g. an MCU reset
    // mid-write).  1 ms at 400 kHz = 400 cycles — well above the minimum.
    common_hal_time_delay_ms(1);

    // CMD7(RCA=0) — best-effort deselect.  If the card is in Transfer or
    // Programming state from a prior session, this moves it back to Stand-by.
    // Ignored by cards already in Idle/Inactive state.
    rp2350_sdio_command_R1(&self->sdio, CMD7, 0, NULL);
    common_hal_time_delay_ms(1);

    // CMD8 — voltage check (VHS = 0x1, check pattern 0xAA).
    for (int retry = 0; retry <= 5; retry++) {
        common_hal_time_delay_ms(1);
        rp2350_sdio_command_R1(&self->sdio, CMD0, 0, NULL);
        st = rp2350_sdio_command_R1(&self->sdio, CMD8, 0x000001AAu, &resp);
        SDIO_INIT_DEBUG("sdioio: CMD8 attempt %d: %s (resp=0x%08lx)\n",
            retry, sdio_status_name(st), (unsigned long)resp);
        if (st != SDIO_ERR_RESPONSE_TIMEOUT) {
            break; // got a real response (OK, CRC, or code error) — proceed
        }
    }
    // Treat CRC/code errors on CMD8 as "old card" (v1) — CMD8 is only
    // mandatory for v2 cards.  A CRC error here likely means the card is in
    // an unexpected state but still alive; proceed with v1/v2 auto-detect via
    // ACMD41.
    bool is_v2 = (st == SDIO_OK && (resp & 0xFFF) == 0x1AA);

    // ACMD41 — send OCR, wait for card to leave idle.
    uint32_t arg41 = OCR_VDD_33;
    if (is_v2) {
        arg41 |= OCR_SDHC; // HCS=1: host supports SDHC/SDXC
    }
    uint64_t deadline = now_ms() + 2000;
    do {
        st = send_acmd(self, ACMD41, arg41, &resp);
        // Tolerate transient errors (timeout, CRC, code) during the busy-wait
        // loop — the card may not respond immediately while it is initialising.
        if (now_ms() > deadline) {
            SDIO_INIT_DEBUG("sdioio: ACMD41 timed out: card busy bit never set (OCR=0x%08lx)\n",
                (unsigned long)resp);
            return -MP_ETIMEDOUT;
        }
        if (st == SDIO_ERR_RESPONSE_TIMEOUT ||
            st == SDIO_ERR_RESPONSE_CRC ||
            st == SDIO_ERR_RESPONSE_CODE) {
            continue;
        }
        if (st != SDIO_OK) {
            SDIO_INIT_DEBUG("sdioio: ACMD41 failed: %s (resp=0x%08lx)\n",
                sdio_status_name(st), (unsigned long)resp);
            return -MP_EIO;
        }
    } while (!(resp & OCR_BUSY));

    // Determine card type.
    self->cdv = (is_v2 && (resp & OCR_SDHC)) ? 1 : 512;

    // CMD2 — get CID (136-bit response; we ignore the payload).
    uint8_t cid[16];
    st = rp2350_sdio_command_R2(&self->sdio, CMD2, 0, cid);
    if (st != SDIO_OK) {
        SDIO_INIT_DEBUG("sdioio: CMD2 (ALL_SEND_CID) failed: %s\n",
            sdio_status_name(st));
        return -MP_EIO;
    }

    // CMD3 — get relative card address.
    st = rp2350_sdio_command_R1(&self->sdio, CMD3, 0, &resp);
    if (st != SDIO_OK) {
        SDIO_INIT_DEBUG("sdioio: CMD3 (SEND_RELATIVE_ADDR) failed: %s\n",
            sdio_status_name(st));
        return -MP_EIO;
    }
    self->rca = (uint16_t)(resp >> 16);

    // CMD9 — read CSD (136-bit response) while card is still in Stand-by state.
    // Must be issued BEFORE CMD7 (which moves card to Transfer state).
    uint8_t csd[16];
    st = rp2350_sdio_command_R2(&self->sdio, 9, (uint32_t)self->rca << 16, csd);
    if (st != SDIO_OK) {
        SDIO_INIT_DEBUG("sdioio: CMD9 (SEND_CSD) failed: %s (RCA=0x%04x)\n",
            sdio_status_name(st), self->rca);
        return -MP_EIO;
    }

    // CMD7 — select card using RCA.
    st = rp2350_sdio_command_R1(&self->sdio, CMD7, (uint32_t)self->rca << 16, &resp);
    if (st != SDIO_OK) {
        SDIO_INIT_DEBUG("sdioio: CMD7 (SELECT_CARD) failed: %s (RCA=0x%04x)\n",
            sdio_status_name(st), self->rca);
        return -MP_EIO;
    }

    // ACMD42 — disable pull-up on DAT3.
    st = send_acmd(self, ACMD42, 0x0, &resp);
    if (st != SDIO_OK) {
        SDIO_INIT_DEBUG("sdioio: ACMD42 (SET_CLR_CARD_DETECT) failed: %s\n",
            sdio_status_name(st));
        return -MP_EIO;
    }

    // ACMD6 — switch to 4-bit bus width.
    if (self->width == 4) {
        st = send_acmd(self, ACMD6, 0x2, &resp); // bus width = 4
        if (st != SDIO_OK) {
            SDIO_INIT_DEBUG("sdioio: ACMD6 (SET_BUS_WIDTH 4-bit) failed: %s — falling back to 1-bit\n",
                sdio_status_name(st));
            // Fall back to 1-bit.
            self->width = 1;
        }
    }

    // CMD16 — set block length to 512.
    st = rp2350_sdio_command_R1(&self->sdio, CMD16, RP2350_SDIO_BLOCK_SIZE, &resp);
    if (st != SDIO_OK) {
        SDIO_INIT_DEBUG("sdioio: CMD16 (SET_BLOCKLEN 512) failed: %s\n",
            sdio_status_name(st));
        return -MP_EIO;
    }
    SDIO_INIT_DEBUG("sdioio: Initialization succeeded");

    // Decode sector count from CSD (read above, before CMD7).
    uint8_t csd_version = (csd[0] >> 6) & 3;
    if (csd_version == 1) {
        // CSD v2 (SDHC/SDXC): sectors = (C_SIZE + 1) * 1024
        uint32_t c_size = (((uint32_t)csd[7] & 0x3F) << 16) |
            ((uint32_t)csd[8] << 8) |
            (uint32_t)csd[9];
        self->sectors = (c_size + 1) * 1024u;
    } else {
        // CSD v1
        uint32_t read_bl_len = csd[5] & 0x0F;
        uint32_t c_size =
            (((uint32_t)csd[6] & 0x03) << 10) |
            ((uint32_t)csd[7] << 2) |
            ((uint32_t)csd[8] >> 6);
        uint32_t c_size_mult =
            (((uint32_t)csd[9] & 0x03) << 1) |
            ((uint32_t)csd[10] >> 7);
        uint32_t block_size = 1u << read_bl_len;
        uint32_t block_count = (c_size + 1) * (1u << (c_size_mult + 2));
        self->sectors = block_count * (block_size / RP2350_SDIO_BLOCK_SIZE);
    }

    return 0;
}

// ---------------------------------------------------------------------------
// common_hal interface
// ---------------------------------------------------------------------------

void common_hal_sdioio_sdcard_construct(sdioio_sdcard_obj_t *self,
    const mcu_pin_obj_t *clock, const mcu_pin_obj_t *command,
    uint8_t num_data, const mcu_pin_obj_t **data, uint32_t frequency) {

    memset(&self->sdio, 0, sizeof(self->sdio));
    self->deinited = false;
    self->never_reset_flag = false;
    self->width = (num_data >= 4) ? 4 : 1;

    uint d0_gpio = data[0]->number;
    uint cmd_gpio = command->number;
    // CLK must be exactly (D0 - 2) mod 32 per PIO program constraint.
    // PIO uses `wait pin SDIO_CLK_PIN_D0_OFFSET` with IN base = D0, so
    // CLK = (D0 + SDIO_CLK_PIN_D0_OFFSET) % 32 = (D0 - 2) mod 32.
    uint clk_gpio = (d0_gpio + SDIO_CLK_PIN_D0_OFFSET) % 32;
    if (clock->number != clk_gpio) {
        mp_raise_ValueError_varg(
            MP_ERROR_TEXT("CLK pin must be %d (D0-2 mod 32)"), clk_gpio);
    }
    // D1-D3 must be consecutive.
    if (num_data >= 4) {
        for (int i = 1; i < 4; i++) {
            if (data[i]->number != d0_gpio + (uint)i) {
                mp_raise_ValueError(MP_ERROR_TEXT("Data pins must be consecutive D0, D0+1, D0+2, D0+3"));
            }
        }
    }

    // Compute PIO clock divider.
    uint32_t sys_hz = clock_get_hz(clk_sys);
    float clk_div = (float)sys_hz / ((float)CLKDIV * (float)frequency);
    if (clk_div < 1.0f) {
        clk_div = 1.0f;
    }
    // Store effective frequency.
    self->frequency = (uint32_t)((float)sys_hz / ((float)CLKDIV * clk_div));

    self->clk_pin_no = clock->number;
    self->cmd_pin_no = command->number;
    self->d0_pin_no = d0_gpio;
    self->num_data = num_data;

    // Claim pins.
    claim_pin(clock);
    claim_pin(command);
    for (int i = 0; i < num_data; i++) {
        claim_pin(data[i]);
    }

    // Slow start: initialise at 400 kHz (per SD spec).
    float init_div = (float)sys_hz / ((float)CLKDIV * 400000.0f);
    SDIO_INIT_DEBUG("sdioio: sys_hz=%lu CLKDIV=%d init_div=%.2f effective_khz=%lu\n",
        (unsigned long)sys_hz, (int)CLKDIV, (double)init_div,
        (unsigned long)((float)sys_hz / ((float)CLKDIV * init_div) / 1000.0f));
    if (!rp2350_sdio_init(&self->sdio, clk_gpio, cmd_gpio, d0_gpio, init_div)) {
        reset_pin_number(self->clk_pin_no);
        reset_pin_number(self->cmd_pin_no);
        for (int i = 0; i < num_data; i++) {
            reset_pin_number(self->d0_pin_no + i);
        }
        mp_raise_OSError(MP_EIO);
    }

    // Card initialisation sequence.
    int result = sdioio_init_card(self);
    if (result < 0) {
        rp2350_sdio_deinit(&self->sdio);
        reset_pin_number(self->clk_pin_no);
        reset_pin_number(self->cmd_pin_no);
        for (int i = 0; i < num_data; i++) {
            reset_pin_number(self->d0_pin_no + i);
        }
        mp_raise_OSError(-result);
    }

    // Switch to requested clock speed.
    rp2350_sdio_set_clkdiv(&self->sdio, clk_div);
}

void common_hal_sdioio_sdcard_deinit(sdioio_sdcard_obj_t *self) {
    if (self->deinited) {
        return;
    }
    rp2350_sdio_deinit(&self->sdio);
    reset_pin_number(self->clk_pin_no);
    reset_pin_number(self->cmd_pin_no);
    for (int i = 0; i < self->num_data; i++) {
        reset_pin_number(self->d0_pin_no + i);
    }
    self->deinited = true;
}

bool common_hal_sdioio_sdcard_deinited(sdioio_sdcard_obj_t *self) {
    return self->deinited;
}

bool common_hal_sdioio_sdcard_configure(sdioio_sdcard_obj_t *self,
    uint32_t baudrate, uint8_t width) {
    if (baudrate != 0) {
        uint32_t sys_hz = clock_get_hz(clk_sys);
        float clk_div = (float)sys_hz / ((float)CLKDIV * (float)baudrate);
        if (clk_div < 1.0f) {
            clk_div = 1.0f;
        }
        self->frequency = (uint32_t)((float)sys_hz / ((float)CLKDIV * clk_div));
        rp2350_sdio_set_clkdiv(&self->sdio, clk_div);
    }
    if (width != 0) {
        self->width = width;
    }
    return true;
}

void common_hal_sdioio_sdcard_unlock(sdioio_sdcard_obj_t *self) {
    // No locking needed for SDIO (single owner).
}

uint32_t common_hal_sdioio_sdcard_get_frequency(sdioio_sdcard_obj_t *self) {
    return self->frequency;
}

uint8_t common_hal_sdioio_sdcard_get_width(sdioio_sdcard_obj_t *self) {
    return self->width;
}

uint32_t common_hal_sdioio_sdcard_get_count(sdioio_sdcard_obj_t *self) {
    return self->sectors;
}

void common_hal_sdioio_sdcard_never_reset(sdioio_sdcard_obj_t *self) {
    self->never_reset_flag = true;
}

// ---------------------------------------------------------------------------
// Block I/O
// ---------------------------------------------------------------------------

mp_negative_errno_t common_hal_sdioio_sdcard_readblocks(
    sdioio_sdcard_obj_t *self, uint32_t start_block, mp_buffer_info_t *bufinfo) {
    if (self->deinited) {
        return -MP_ENODEV;
    }
    uint32_t num_blocks = (uint32_t)(bufinfo->len / RP2350_SDIO_BLOCK_SIZE);
    if (num_blocks == 0 || bufinfo->len % RP2350_SDIO_BLOCK_SIZE != 0) {
        return -MP_EINVAL;
    }
    if (num_blocks > RP2350_SDIO_MAX_BLOCKS) {
        return -MP_EINVAL;
    }

    uint32_t addr = (self->cdv == 1) ? start_block : start_block * RP2350_SDIO_BLOCK_SIZE;
    uint32_t resp = 0;
    uint8_t cmd = (num_blocks == 1) ? CMD17 : CMD18;
    sdio_status_t st = rp2350_sdio_command_R1(&self->sdio, cmd, addr, &resp);
    if (st != SDIO_OK) {
        return -MP_EIO;
    }

    st = rp2350_sdio_rx_start(&self->sdio, (uint8_t *)bufinfo->buf, num_blocks);
    if (st != SDIO_OK) {
        return -MP_EIO;
    }

    // Poll until complete.
    while ((st = rp2350_sdio_rx_poll(&self->sdio)) == SDIO_BUSY) {
        RUN_BACKGROUND_TASKS;
    }

    if (num_blocks > 1) {
        // Send CMD12 to stop transmission.
        rp2350_sdio_command_R1(&self->sdio, CMD12, 0, &resp);
    }

    if (st != SDIO_OK) {
        return (st == SDIO_ERR_DATA_CRC) ? -MP_EIO : -MP_ETIMEDOUT;
    }
    return 0;
}

mp_negative_errno_t common_hal_sdioio_sdcard_writeblocks(
    sdioio_sdcard_obj_t *self, uint32_t start_block, mp_buffer_info_t *bufinfo) {
    if (self->deinited) {
        return -MP_ENODEV;
    }
    uint32_t num_blocks = (uint32_t)(bufinfo->len / RP2350_SDIO_BLOCK_SIZE);
    if (num_blocks == 0 || bufinfo->len % RP2350_SDIO_BLOCK_SIZE != 0) {
        return -MP_EINVAL;
    }
    if (num_blocks > RP2350_SDIO_MAX_BLOCKS) {
        return -MP_EINVAL;
    }

    uint32_t addr = (self->cdv == 1) ? start_block : start_block * RP2350_SDIO_BLOCK_SIZE;
    uint32_t resp = 0;
    uint8_t cmd = (num_blocks == 1) ? CMD24 : CMD25;
    sdio_status_t st = rp2350_sdio_command_R1(&self->sdio, cmd, addr, &resp);
    if (st != SDIO_OK) {
        return -MP_EIO;
    }

    st = rp2350_sdio_tx_start(&self->sdio, (const uint8_t *)bufinfo->buf, num_blocks);
    if (st != SDIO_OK) {
        return -MP_EIO;
    }

    while ((st = rp2350_sdio_tx_poll(&self->sdio)) == SDIO_BUSY) {
        RUN_BACKGROUND_TASKS;
    }

    if (num_blocks > 1) {
        rp2350_sdio_command_R1(&self->sdio, CMD12, 0, &resp);
    }

    if (st == SDIO_ERR_WRITE_CRC || st == SDIO_ERR_WRITE_FAIL) {
        return -MP_EIO;
    }
    if (st == SDIO_ERR_DATA_TIMEOUT) {
        return -MP_ETIMEDOUT;
    }
    return (st == SDIO_OK) ? 0 : -MP_EIO;
}

// ---------------------------------------------------------------------------
// Native VFS blockdev helpers
// ---------------------------------------------------------------------------

mp_negative_errno_t sdioio_sdcard_readblocks(mp_obj_t self_in, uint8_t *buf,
    uint32_t start_block, uint32_t buflen) {
    sdioio_sdcard_obj_t *self = (sdioio_sdcard_obj_t *)self_in;
    mp_buffer_info_t info = {.buf = buf, .len = buflen};
    return common_hal_sdioio_sdcard_readblocks(self, start_block, &info);
}

mp_negative_errno_t sdioio_sdcard_writeblocks(mp_obj_t self_in, uint8_t *buf,
    uint32_t start_block, uint32_t buflen) {
    sdioio_sdcard_obj_t *self = (sdioio_sdcard_obj_t *)self_in;
    mp_buffer_info_t info = {.buf = (void *)buf, .len = buflen};
    return common_hal_sdioio_sdcard_writeblocks(self, start_block, &info);
}

bool sdioio_sdcard_ioctl(mp_obj_t self_in, size_t cmd, size_t arg,
    mp_int_t *out_value) {
    sdioio_sdcard_obj_t *self = (sdioio_sdcard_obj_t *)self_in;
    switch (cmd) {
        case MP_BLOCKDEV_IOCTL_INIT:
            *out_value = 0;
            return true;
        case MP_BLOCKDEV_IOCTL_DEINIT:
            *out_value = 0;
            return true;
        case MP_BLOCKDEV_IOCTL_SYNC:
            *out_value = 0;
            return true;
        case MP_BLOCKDEV_IOCTL_BLOCK_COUNT:
            *out_value = (mp_int_t)self->sectors;
            return true;
        case MP_BLOCKDEV_IOCTL_BLOCK_SIZE:
            *out_value = RP2350_SDIO_BLOCK_SIZE;
            return true;
        default:
            return false;
    }
}
