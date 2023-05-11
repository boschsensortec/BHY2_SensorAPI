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
 * @file    common_callbacks.h
 * @brief   Header file for the command line utility callbacks
 *
 */

#ifndef _COMMON_CALLBACKS_H_
#define _COMMON_CALLBACKS_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>

#include "cli.h"

#define CLI_STREAM_BUF_MAX  240
#define WRITE_TIMEOUT_MS    10000

typedef struct
{
    FILE *wfp;
    int32_t data_len;
    bool wrAck;
    uint32_t last_write_ts;
} write_file;

#ifndef PC

int8_t cls_help(void *ref);

int8_t cls_callback(uint8_t argc, uint8_t *argv[], void *ref);

int8_t echo_help(void *ref);

int8_t echo_callback(uint8_t argc, uint8_t *argv[], void *ref);

int8_t heartbeat_help(void *ref);

int8_t heartbeat_callback(uint8_t argc, uint8_t *argv[], void *ref);

int8_t streambuff_help(void *ref);

int8_t streambuff_callback(uint8_t argc, uint8_t *argv[], void *ref);

int8_t mklog_help(void *ref);

int8_t mklog_callback(uint8_t argc, uint8_t *argv[], void *ref);

int8_t rm_help(void *ref);

int8_t rm_callback(uint8_t argc, uint8_t *argv[], void *ref);

int8_t ls_help(void *ref);

int8_t ls_callback(uint8_t argc, uint8_t *argv[], void *ref);

int8_t wrfile_help(void *ref);

int8_t wrfile_callback(uint8_t argc, uint8_t *argv[], void *ref);

int8_t rdfile_help(void *ref);

int8_t rdfile_callback(uint8_t argc, uint8_t *argv[], void *ref);

int8_t cls_help(void *ref);

int8_t cls_callback(uint8_t argc, uint8_t *argv[], void *ref);

#endif

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* _COMMON_CALLBACKS_H_ */
