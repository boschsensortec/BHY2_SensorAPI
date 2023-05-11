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
 * @file    cli.h
 * @brief   Header file for the command line utility
 *
 */

#ifndef _CLI_H_
#define _CLI_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>
#include <stddef.h>

#ifndef CLI_CMD_LEN
#define CLI_CMD_LEN          16
#endif

#define CLI_OK               INT8_C(0)
#define CLI_E_NULL_PTR       INT8_C(-1)
#define CLI_E_UNKNOWN_CMD    INT8_C(-2)
#define CLI_E_INVALID_PARAM  INT8_C(-3)

typedef int8_t (*cli_callback_t)(uint8_t argc, uint8_t *argv[], void *ref);

typedef int8_t (*cli_help_callback_t)(void *ref);

typedef struct
{
    uint8_t cmd_char;
    uint8_t cmd_str[CLI_CMD_LEN];
    uint8_t n_args;
    cli_callback_t callback;
    cli_help_callback_t help_callback;
} cli_callback_table_t;

typedef struct
{
    void *ref;
    uint8_t n_cmds;
    cli_callback_table_t *table;
} cli_dev_t;

int8_t cli_run(uint8_t argc, uint8_t *argv[], const cli_dev_t *dev);

int8_t cli_help(void *ref, const cli_dev_t *dev);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* _CLI_H_ */
