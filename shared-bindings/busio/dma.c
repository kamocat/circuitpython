// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2026
//
// SPDX-License-Identifier: MIT

#include "py/runtime.h"

#include "shared-bindings/busio/dma.h"
#include "shared-bindings/busio/I2C.h"
#include "shared-bindings/busio/SPI.h"
#include "shared-bindings/busio/UART.h"

#include "shared/runtime/buffer_helper.h"

#include "hardware/dma.h"
#include "pico/error.h"

#if CIRCUITPY_BUSIO_DMA

static mp_negative_errno_t _pico_to_mp_error(int result) {
    switch (result) {
        case PICO_ERROR_GENERIC:
            return -MP_ENODEV;
        case PICO_ERROR_TIMEOUT:
            return -MP_ETIMEDOUT;
        case PICO_ERROR_INSUFFICIENT_RESOURCES:
            return -MP_EBUSY;
        default:
            return -MP_EIO;
    }
}

static uint _check_dma_channel_or_raise(uint dma_channel) {
    if (dma_channel >= NUM_DMA_CHANNELS) {
        mp_raise_OSError(-_pico_to_mp_error((int)dma_channel));
    }
    return dma_channel;
}

static void _check_i2c_lock(busio_i2c_obj_t *i2c) {
    if (!common_hal_busio_i2c_has_lock(i2c)) {
        mp_raise_RuntimeError(MP_ERROR_TEXT("Function requires lock"));
    }
}

static void _check_spi_lock(busio_spi_obj_t *spi) {
    if (!common_hal_busio_spi_has_lock(spi)) {
        mp_raise_RuntimeError(MP_ERROR_TEXT("Function requires lock"));
    }
}

static mp_obj_t busio_dma_i2c_read(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_i2c, ARG_address, ARG_buffer, ARG_end };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_i2c, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_address, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_buffer, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_end, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    busio_i2c_obj_t *i2c = mp_arg_validate_type(args[ARG_i2c].u_obj, &busio_i2c_type, MP_QSTR_i2c);
    _check_i2c_lock(i2c);

    mp_int_t address = args[ARG_address].u_int;
    mp_arg_validate_int_range(address, 0, 0x7f, MP_QSTR_address);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_buffer].u_obj, &bufinfo, MP_BUFFER_WRITE);

    uint dma_channel = common_hal_busio_dma_i2c_read(i2c, address, bufinfo.buf, bufinfo.len, args[ARG_end].u_bool);
    return mp_obj_new_int_from_uint(_check_dma_channel_or_raise(dma_channel));
}
MP_DEFINE_CONST_FUN_OBJ_KW(busio_dma_i2c_read_obj, 0, busio_dma_i2c_read);

static mp_obj_t busio_dma_i2c_write(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_i2c, ARG_address, ARG_buffer, ARG_end };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_i2c, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_address, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_buffer, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_end, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    busio_i2c_obj_t *i2c = mp_arg_validate_type(args[ARG_i2c].u_obj, &busio_i2c_type, MP_QSTR_i2c);
    _check_i2c_lock(i2c);

    mp_int_t address = args[ARG_address].u_int;
    mp_arg_validate_int_range(address, 0, 0x7f, MP_QSTR_address);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_buffer].u_obj, &bufinfo, MP_BUFFER_READ);

    uint dma_channel = common_hal_busio_dma_i2c_write(i2c, address, bufinfo.buf, bufinfo.len, args[ARG_end].u_bool);
    return mp_obj_new_int_from_uint(_check_dma_channel_or_raise(dma_channel));
}
MP_DEFINE_CONST_FUN_OBJ_KW(busio_dma_i2c_write_obj, 0, busio_dma_i2c_write);

static mp_obj_t busio_dma_i2c_is_busy(mp_obj_t dma_channel_obj) {
    return mp_obj_new_bool(common_hal_busio_dma_i2c_is_busy(mp_obj_get_int(dma_channel_obj)));
}
MP_DEFINE_CONST_FUN_OBJ_1(busio_dma_i2c_is_busy_obj, busio_dma_i2c_is_busy);

static mp_obj_t busio_dma_spi_write(mp_obj_t spi_obj, mp_obj_t buffer_obj) {
    busio_spi_obj_t *spi = mp_arg_validate_type(spi_obj, &busio_spi_type, MP_QSTR_spi);
    _check_spi_lock(spi);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buffer_obj, &bufinfo, MP_BUFFER_READ);

    uint dma_channel = common_hal_busio_dma_spi_write(spi, bufinfo.buf, bufinfo.len);
    return mp_obj_new_int_from_uint(_check_dma_channel_or_raise(dma_channel));
}
MP_DEFINE_CONST_FUN_OBJ_2(busio_dma_spi_write_obj, busio_dma_spi_write);

static mp_obj_t busio_dma_spi_readinto(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_spi, ARG_buffer, ARG_write_value };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_spi, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_buffer, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_write_value, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    busio_spi_obj_t *spi = mp_arg_validate_type(args[ARG_spi].u_obj, &busio_spi_type, MP_QSTR_spi);
    _check_spi_lock(spi);

    mp_int_t write_value = args[ARG_write_value].u_int;
    mp_arg_validate_int_range(write_value, 0, 0xff, MP_QSTR_write_value);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_buffer].u_obj, &bufinfo, MP_BUFFER_WRITE);

    uint dma_channel = common_hal_busio_dma_spi_read(spi, write_value, bufinfo.buf, bufinfo.len);
    return mp_obj_new_int_from_uint(_check_dma_channel_or_raise(dma_channel));
}
MP_DEFINE_CONST_FUN_OBJ_KW(busio_dma_spi_readinto_obj, 0, busio_dma_spi_readinto);

static mp_obj_t busio_dma_spi_write_readinto(mp_obj_t spi_obj, mp_obj_t out_buffer_obj, mp_obj_t in_buffer_obj) {
    busio_spi_obj_t *spi = mp_arg_validate_type(spi_obj, &busio_spi_type, MP_QSTR_spi);
    _check_spi_lock(spi);

    mp_buffer_info_t out_bufinfo;
    mp_get_buffer_raise(out_buffer_obj, &out_bufinfo, MP_BUFFER_READ);

    mp_buffer_info_t in_bufinfo;
    mp_get_buffer_raise(in_buffer_obj, &in_bufinfo, MP_BUFFER_WRITE);

    if (out_bufinfo.len != in_bufinfo.len) {
        mp_raise_ValueError(MP_ERROR_TEXT("buffers must be same length"));
    }

    uint dma_channel = common_hal_busio_dma_spi_transfer(spi, out_bufinfo.buf, in_bufinfo.buf, out_bufinfo.len);
    return mp_obj_new_int_from_uint(_check_dma_channel_or_raise(dma_channel));
}
MP_DEFINE_CONST_FUN_OBJ_3(busio_dma_spi_write_readinto_obj, busio_dma_spi_write_readinto);

static mp_obj_t busio_dma_spi_is_busy(mp_obj_t dma_channel_obj) {
    return mp_obj_new_bool(common_hal_busio_dma_spi_is_busy(mp_obj_get_int(dma_channel_obj)));
}
MP_DEFINE_CONST_FUN_OBJ_1(busio_dma_spi_is_busy_obj, busio_dma_spi_is_busy);

static mp_obj_t busio_dma_uart_readinto(mp_obj_t uart_obj, mp_obj_t buffer_obj) {
    busio_uart_obj_t *uart = mp_arg_validate_type(uart_obj, &busio_uart_type, MP_QSTR_uart);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buffer_obj, &bufinfo, MP_BUFFER_WRITE);

    uint dma_channel = common_hal_busio_dma_uart_read(uart, bufinfo.buf, bufinfo.len);
    return mp_obj_new_int_from_uint(_check_dma_channel_or_raise(dma_channel));
}
MP_DEFINE_CONST_FUN_OBJ_2(busio_dma_uart_readinto_obj, busio_dma_uart_readinto);

static mp_obj_t busio_dma_uart_write(mp_obj_t uart_obj, mp_obj_t buffer_obj) {
    busio_uart_obj_t *uart = mp_arg_validate_type(uart_obj, &busio_uart_type, MP_QSTR_uart);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buffer_obj, &bufinfo, MP_BUFFER_READ);

    uint dma_channel = common_hal_busio_dma_uart_write(uart, bufinfo.buf, bufinfo.len);
    return mp_obj_new_int_from_uint(_check_dma_channel_or_raise(dma_channel));
}
MP_DEFINE_CONST_FUN_OBJ_2(busio_dma_uart_write_obj, busio_dma_uart_write);

static mp_obj_t busio_dma_uart_is_busy(mp_obj_t dma_channel_obj) {
    return mp_obj_new_bool(common_hal_busio_dma_uart_is_busy(mp_obj_get_int(dma_channel_obj)));
}
MP_DEFINE_CONST_FUN_OBJ_1(busio_dma_uart_is_busy_obj, busio_dma_uart_is_busy);

static const mp_rom_map_elem_t busio_dma_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_dma) },

    { MP_ROM_QSTR(MP_QSTR_i2c_read), MP_ROM_PTR(&busio_dma_i2c_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_i2c_write), MP_ROM_PTR(&busio_dma_i2c_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_i2c_is_busy), MP_ROM_PTR(&busio_dma_i2c_is_busy_obj) },

    { MP_ROM_QSTR(MP_QSTR_spi_write), MP_ROM_PTR(&busio_dma_spi_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_spi_readinto), MP_ROM_PTR(&busio_dma_spi_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_spi_write_readinto), MP_ROM_PTR(&busio_dma_spi_write_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_spi_is_busy), MP_ROM_PTR(&busio_dma_spi_is_busy_obj) },

    { MP_ROM_QSTR(MP_QSTR_uart_readinto), MP_ROM_PTR(&busio_dma_uart_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_uart_write), MP_ROM_PTR(&busio_dma_uart_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_uart_is_busy), MP_ROM_PTR(&busio_dma_uart_is_busy_obj) },
};

static MP_DEFINE_CONST_DICT(busio_dma_module_globals, busio_dma_module_globals_table);

const mp_obj_module_t busio_dma_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&busio_dma_module_globals,
};

#endif
