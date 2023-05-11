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
 * @file    parse.h
 * @brief   Header file for the parse functions for the command line utility
 *
 */

#ifndef BHY2CLI_PARSE_H_
#define BHY2CLI_PARSE_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdbool.h>

#include "bhy2.h"
#include "bhy2_klio.h"
#include "logbin.h"

#define PARSE_FLAG_NONE              UINT8_C(0x00)
#define PARSE_FLAG_STREAM            UINT8_C(0x01)
#define PARSE_FLAG_LOG               UINT8_C(0x02)
#define PARSE_FLAG_HEXSTREAM         UINT8_C(0x04)

#define PARSE_SET_FLAG(var, flag)    (var | flag)
#define PARSE_CLEAR_FLAG(var, flag)  (var & ~flag)

struct parse_sensor_details
{
    uint8_t id;
    uint8_t accuracy;
    float scaling_factor;
    uint8_t parse_flag;
};

struct parse_ref
{
    struct parse_sensor_details sensor[BHY2_MAX_SIMUL_SENSORS];
    struct bhy2_dev *bhy2;
    struct logbin_dev logdev;
};

struct parse_sensor_details *parse_get_sensor_details(uint8_t id, struct parse_ref *ref);

struct parse_sensor_details *parse_add_sensor_details(uint8_t id, struct parse_ref *ref);

void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

void parse_3axis_s16(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

void parse_euler(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

void parse_quaternion(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

void parse_s16_as_float(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

void parse_scalar_u32(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

void parse_scalar_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

void parse_activity(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

void parse_u16_as_float(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

void parse_u24_as_float(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

void parse_proximity(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

void parse_scalar_u8(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

void parse_generic(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

void parse_device_ori(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

void parse_gps(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

void parse_debug_message(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* BHY2CLI_PARSE_H_ */
