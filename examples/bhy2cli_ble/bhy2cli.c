/**
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file    bhy2cli.c
 * @brief   Command line utility for the BHI260/BHA260
 *
 */

#ifdef __STDC_ALLOC_LIB__
#define __STDC_WANT_LIB_EXT2__  1
#else
#define _POSIX_C_SOURCE         200809L
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>
#include <ctype.h>

#include "bhy2.h"
#include "bhy2_parse.h"
#include "parse.h"
#include "cli.h"
#include "common.h"
#include "coines.h"
#include "bhy2cli_callbacks.h"
#include "common_callbacks.h"
#include "verbose.h"

#define LIFE_LED_PERIOD_MS  UINT32_C(2500)
#define LIFE_LED_DUR_MS     UINT32_C(50)

bool echo_on = true;
bool heartbeat_on = false;
uint16_t stream_buff_len = 0;
#ifndef PC
static uint8_t out_buff[CLI_STREAM_BUF_MAX] = { 0 };
#endif
static uint16_t out_idx = 0;
#ifdef PC
static volatile bool end_streaming = false;
#endif
#ifndef PC
static uint8_t *argv[50] = { 0 };
static uint8_t argc = 0;
uint8_t inp[2048] = { 0 };
enum coines_comm_intf bhy2cli_intf = COINES_COMM_INTF_BLE;
#endif
static struct bhy2_cli_ref cli_ref = { 0 };

bool cmd_in_process = false;

#ifdef PC
void sigint_handler(int sig_num)
{
    (void)sig_num;
    (void)signal(SIGINT, NULL);

    if (end_streaming)
    {
        INFO("Force exit\r\n");
        exit(0);
    }

    INFO("\r\nExiting\r\n");
    end_streaming = true;
}
#endif

#ifdef PC
int main(int argc, char *argv[])
{
    (void)signal(SIGINT, sigint_handler);
    bool first_run = true;
#else
int main(void)
{
    uint16_t inp_idx, bytes_read, bytes_read_now;
    uint8_t nl = '\n';

#endif
#ifndef PC
    uint32_t blink_on = 0, blink_off = 0;
    bool led_state = false;
#endif

    cli_ref.cli_dev.n_cmds = bhy2_get_n_cli_callbacks();
    cli_ref.cli_dev.ref = &cli_ref;
    cli_ref.cli_dev.table = bhy2_get_cli_callbacks();
    cli_ref.parse_table.bhy2 = &cli_ref.bhy2;
    int8_t cli_ret;
    enum bhy2_intf intf;
#ifdef BHY2_USE_I2C
    intf = BHY2_I2C_INTERFACE;
#else
    intf = BHY2_SPI_INTERFACE;
#endif

    /* Execution starts here */
#ifdef PC
    setup_interfaces(false, intf);
#else
    setup_interfaces(true, intf);
#endif

    /* Disable printf buffering for stdout */
    setbuf(stdout, NULL);

#ifndef PC
    if (coines_intf_connected(COINES_COMM_INTF_BLE))
    {
        bhy2cli_intf = COINES_COMM_INTF_BLE;
    }
    else
    {
        bhy2cli_intf = COINES_COMM_INTF_USB;
    }

    /* Read and discard any data from the buffer */
    if (coines_intf_available(bhy2cli_intf))
    {
        bytes_read_now = coines_read_intf(bhy2cli_intf, inp, coines_intf_available(bhy2cli_intf));
    }

    PRINT("\033c\033[2J"); /* Clear screen */
    PRINT("Type help to view the list of available commands\r\n");
#endif

    bhy2_callbacks_init(&cli_ref);

#ifdef PC
    while (bhy2_are_sensors_active() || first_run)
    {
        first_run = false;
        if (end_streaming)
        {
            break;
        }

#else
    bytes_read = 0;
    bytes_read_now = 0;
    cmd_in_process = true;

    while (1)
    {
#endif

#ifndef PC
        if (coines_intf_connected(COINES_COMM_INTF_BLE))
        {
            bhy2cli_intf = COINES_COMM_INTF_BLE;
        }
        else
        {
            bhy2cli_intf = COINES_COMM_INTF_USB;
        }

        /* Read data from the service and append into the inp buffer */
        if (coines_intf_available(bhy2cli_intf))
        {
            bytes_read_now = coines_read_intf(bhy2cli_intf, &inp[bytes_read], coines_intf_available(bhy2cli_intf));

            /* Only for use with the terminal. Comment the following line otherwise */
            if (echo_on)
            {
                coines_write_intf(bhy2cli_intf, &inp[bytes_read], bytes_read_now); /* Echo back the input */
            }

            bytes_read += bytes_read_now;
            cmd_in_process = true;

            /* Check the inp buffer for a termination character. */
            for (uint16_t i = 0; i < bytes_read; i++)
            {
                if ((inp[i] == '\r') || (inp[i] == '\n'))
                {
                    cmd_in_process = false;

                    /* Only for use with the terminal. Comment the following line otherwise */
                    if (echo_on)
                    {
                        coines_write_intf(bhy2cli_intf, &nl, 1);
                    }
                }
            }
        }

        /* Split the inp buffer into argv[]. */
        if (!cmd_in_process)
        {
            cmd_in_process = true;
            argc = 0;
            argv[argc++] = &inp[0];

            for (inp_idx = 0; inp_idx < bytes_read; inp_idx++)
            {
                /* Wait for the termination of an argument */
                if ((inp[inp_idx] != ' ') && (inp[inp_idx] != '\r') && (inp[inp_idx] != '\n') && (inp_idx < bytes_read))
                {
                    continue;
                }

                /* Skip over whitespace, replacing it with '\0'. */
                while (((inp[inp_idx] == ' ') || (inp[inp_idx] == '\r') || (inp[inp_idx] == '\n')) &&
                       (inp_idx < bytes_read))
                {
                    inp[inp_idx] = '\0';
                    inp_idx++;
                }

                /* Are more arguments present? */
                if (inp_idx < bytes_read)
                {
                    argv[argc++] = &inp[inp_idx];
                }
            }

            bytes_read = 0;
        }

#endif

        /* If there are arguments, process them */
        if (argc)
        {
            cli_ret = cli_run(argc, (uint8_t **)argv, &cli_ref.cli_dev);
            if (cli_ret)
            {
                ERROR("Invalid command. Type help for detailed overview of available commands\r\n\r\n", cli_ret);
            }

            argc = 0;
        }

        if (stream_buff_len && out_idx)
        {
#ifndef PC
            coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_ON);
            coines_write_intf(bhy2cli_intf, out_buff, out_idx);
            out_idx = 0;
            coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_OFF);
#endif
        }

        bhy2_data_parse_callback(&cli_ref);
#ifdef PC
        if (!bhy2_are_sensors_active())
        {
            break;
        }

#endif
#ifndef PC
        if (blink_on <= coines_get_millis())
        {
            blink_on = coines_get_millis() + LIFE_LED_PERIOD_MS;
            blink_off = coines_get_millis() + LIFE_LED_DUR_MS;
            coines_set_led(COINES_LED_RED, COINES_LED_STATE_ON);
            led_state = true;
            if (heartbeat_on)
            {
                PRINT("[H]%u\r\n", coines_get_millis());
                coines_flush_intf(bhy2cli_intf);
            }
        }

        if (led_state && (blink_off <= coines_get_millis()))
        {
            coines_set_led(COINES_LED_RED, COINES_LED_STATE_OFF);
            led_state = false;
            if (heartbeat_on)
            {
                PRINT("[H]%u\r\n", coines_get_millis());
                coines_flush_intf(bhy2cli_intf);
            }
        }

#endif

#ifdef PC
        if (end_streaming)
        {
            bhy2_exit(&cli_ref);
        }

#endif
    }

    close_interfaces(intf);

    return 0;
}

void verbose_write(uint8_t *buffer, uint16_t length)
{
#ifndef PC
    if (stream_buff_len)
    {
        if ((out_idx + length) > sizeof(out_buff))
        {
            coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_ON);
            coines_write_intf(bhy2cli_intf, out_buff, out_idx);
            out_idx = 0;
            coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_OFF);
        }

        memcpy(&out_buff[out_idx], buffer, length);
        out_idx += length;
    }
    else
    {
        coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_ON);
        coines_write_intf(bhy2cli_intf, buffer, length);
        coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_OFF);
    }

#endif
}
