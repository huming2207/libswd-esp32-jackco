/*
 * Serial Wire Debug Open Library.
 * ESP32 SPI driver 
 *
 * Copyright (C) 2018, Paul Freund
 * Copyright (C) 2021, Jackson Ming Hu <huming2207@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the Tomasz Boleslaw CEDRO nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.*
 *
 * Written by Tomasz Boleslaw CEDRO <cederom@tlen.pl>, 2010-2014;
 *
 */

#define DIRECTION_GPIO GPIO_NUM_4
#define DIRECTION_MISO 0
#define DIRECTION_MOSI 1

#define direction_set(gpio, direction) gpio_set_level(gpio, direction)
//#define direction_set(gpio, direction)

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <libswd.h>
#include <esp_log.h>

#define TAG "libswd"

inline void spi_transmit(spi_device_handle_t spi_dev, spi_transaction_t *spi_trans)
{
    if (ESP_OK != spi_device_transmit(spi_dev, spi_trans)) {
        ESP_LOGD(TAG, " - FAIL\n");
    }
}

extern int libswd_drv_mosi_8(libswd_ctx_t *libswdctx, libswd_cmd_t *cmd, char *data, int bits, int nLSBfirst)
{
    ESP_LOGD(TAG, "[MOSI08][%02d] -> ", bits);

    spi_device_handle_t *spi_dev = (spi_device_handle_t *) libswdctx->driver->device;

    spi_transaction_t spi_trans;
    memset(&spi_trans, 0, sizeof(spi_transaction_t));

    spi_trans.flags = SPI_TRANS_USE_TXDATA;
    spi_trans.cmd = 0; // If want to set, change command_bits
    spi_trans.addr = 0; // If want to set, change address_bits
    spi_trans.length = bits; // Bits
    spi_trans.rxlength = 0;
    spi_trans.tx_data[0] = *data;

    direction_set(DIRECTION_GPIO, DIRECTION_MOSI);
    spi_transmit(*spi_dev, &spi_trans);
    direction_set(DIRECTION_GPIO, DIRECTION_MISO);

    ESP_LOGD(TAG, "%02x", spi_trans.tx_data[0]);

    ESP_LOGD(TAG, ";\n");
    return LIBSWD_OK;
}

extern int libswd_drv_mosi_32(libswd_ctx_t *libswdctx, libswd_cmd_t *cmd, int *data, int bits, int nLSBfirst)
{
    ESP_LOGD(TAG, "[MOSI32][%02d] -> ", bits);

    spi_device_handle_t *spi_dev = (spi_device_handle_t *) libswdctx->driver->device;

    spi_transaction_t spi_trans;
    memset(&spi_trans, 0, sizeof(spi_transaction_t));

    spi_trans.flags = SPI_TRANS_USE_TXDATA;
    spi_trans.cmd = 0; // If want to set, change command_bits
    spi_trans.addr = 0; // If want to set, change address_bits
    spi_trans.length = bits; // Bits
    spi_trans.rxlength = 0;

    *((int *) (&spi_trans.tx_data)) = (*data);

    direction_set(DIRECTION_GPIO, DIRECTION_MOSI);
    spi_transmit(*spi_dev, &spi_trans);
    direction_set(DIRECTION_GPIO, DIRECTION_MISO);

    ESP_LOGD(TAG, "%02x %02x %02x %02x", spi_trans.tx_data[0], spi_trans.tx_data[1], spi_trans.tx_data[2],
             spi_trans.tx_data[3]);

    ESP_LOGD(TAG, ";\n");
    return LIBSWD_OK;
}

extern int libswd_drv_miso_8(libswd_ctx_t *libswdctx, libswd_cmd_t *cmd, char *data, int bits, int nLSBfirst)
{
    ESP_LOGD(TAG, "[MISO08][%02d] <- ", bits);

    spi_device_handle_t *spi_dev = (spi_device_handle_t *) libswdctx->driver->device;

    spi_transaction_t spi_trans;
    memset(&spi_trans, 0, sizeof(spi_transaction_t));

    spi_trans.flags = SPI_TRANS_USE_RXDATA;
    spi_trans.cmd = 0; // If want to set, change command_bits
    spi_trans.addr = 0; // If want to set, change address_bits
    spi_trans.length = 0; // Bits
    spi_trans.rxlength = bits; // Bits expected

    spi_transmit(*spi_dev, &spi_trans);

    ESP_LOGD(TAG, "%02x", spi_trans.rx_data[0]);

    (*data) = spi_trans.rx_data[0];

    ESP_LOGD(TAG, ";\n");
    return LIBSWD_OK;
}

extern int libswd_drv_miso_32(libswd_ctx_t *libswdctx, libswd_cmd_t *cmd, int *data, int bits, int nLSBfirst)
{
    ESP_LOGD(TAG, "[MISO32][%02d] <- ", bits);

    spi_device_handle_t *spi_dev = (spi_device_handle_t *) libswdctx->driver->device;

    spi_transaction_t spi_trans;
    memset(&spi_trans, 0, sizeof(spi_transaction_t));

    spi_trans.flags = SPI_TRANS_USE_RXDATA;
    spi_trans.cmd = 0; // If want to set, change command_bits
    spi_trans.addr = 0; // If want to set, change address_bits
    spi_trans.length = 0; // Bits
    spi_trans.rxlength = bits; // Bits expected

    spi_transmit(*spi_dev, &spi_trans);


    (*data) = *((int *) (&spi_trans.rx_data));

    return LIBSWD_OK;
}

extern int libswd_drv_mosi_trn(libswd_ctx_t *libswdctx, int clks)
{
    ESP_LOGD(TAG, "[MOSITN][%02d]", clks);

    if (clks == 0) { return LIBSWD_OK; }

    spi_device_handle_t *spi_dev = (spi_device_handle_t *) libswdctx->driver->device;

    spi_transaction_t spi_trans;
    memset(&spi_trans, 0, sizeof(spi_transaction_t));

    spi_trans.flags = SPI_TRANS_USE_TXDATA;
    spi_trans.cmd = 0; // If want to set, change command_bits
    spi_trans.addr = 0; // If want to set, change address_bits
    spi_trans.length = clks; // Bits
    spi_trans.rxlength = 0;

    direction_set(DIRECTION_GPIO, DIRECTION_MOSI);
    spi_transmit(*spi_dev, &spi_trans);
    direction_set(DIRECTION_GPIO, DIRECTION_MISO);

    ESP_LOGD(TAG, ";\n");
    return LIBSWD_OK;
}

extern int libswd_drv_miso_trn(libswd_ctx_t *libswdctx, int clks)
{
    ESP_LOGD(TAG, "[MISOTN][%02d]", clks);

    if (clks == 0) { return LIBSWD_OK; }
    spi_device_handle_t *spi_dev = (spi_device_handle_t *) libswdctx->driver->device;

    spi_transaction_t spi_trans;
    memset(&spi_trans, 0, sizeof(spi_transaction_t));

    spi_trans.flags = SPI_TRANS_USE_TXDATA;
    spi_trans.cmd = 0; // If want to set, change command_bits
    spi_trans.addr = 0; // If want to set, change address_bits
    spi_trans.length = clks;
    spi_trans.rxlength = 0;

    direction_set(DIRECTION_GPIO, DIRECTION_MOSI);
    spi_transmit(*spi_dev, &spi_trans);
    direction_set(DIRECTION_GPIO, DIRECTION_MISO);

    ESP_LOGD(TAG, ";\n");

    return LIBSWD_OK;
}

extern int libswd_log(libswd_ctx_t *libswdctx, libswd_loglevel_t loglevel, char *msg, ...)
{
    va_list args;
    va_start (args, msg);

    char buffer[256];
    int length = 0;
    length = vsnprintf(buffer, 255, msg, args);
    buffer[length] = '\0';

    switch (loglevel) {
        case LIBSWD_LOGLEVEL_DEBUG: {
            ESP_LOGD(TAG, "%s", buffer);
            break;
        }

        case LIBSWD_LOGLEVEL_MIN: {
            break;
        }

        case LIBSWD_LOGLEVEL_NORMAL: {
            ESP_LOGI(TAG, "%s", buffer);
            break;
        }

        case LIBSWD_LOGLEVEL_ERROR: {
            ESP_LOGE(TAG, "%s", buffer);
            break;
        }

        case LIBSWD_LOGLEVEL_WARNING: {
            ESP_LOGW(TAG, "%s", buffer);
            break;
        }

        case LIBSWD_LOGLEVEL_INFO: {
            ESP_LOGI(TAG, "%s", buffer);
            break;
        }

        case LIBSWD_LOGLEVEL_PAYLOAD: {
            ESP_LOGD(TAG, "%s", buffer);
            break;
        }

        default: {
            break;
        }
    }

    va_end (args);

    return LIBSWD_OK;
}

extern int libswd_log_level_inherit(libswd_ctx_t *libswdctx, int loglevel)
{
    return LIBSWD_OK;
}