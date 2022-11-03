/**
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
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
 * @file    dinject.h
 * @brief   Header file for the data injection functions for the command line utility
 *
 */

#ifndef BHY2CLI_DINJECT_H_
#define BHY2CLI_DINJECT_H_

#include <stdbool.h>

#include "bhy2.h"
#include "bhy2_klio.h"
#include "logbin.h"

#define DINJECT_FAILED         INT8_C(-1)
#define DINJECT_SUCCESSFUL     INT8_C(1)
#define DINJECT_IN_PROGRESS    INT8_C(0)

#define MAX_SAMPLES_PER_BLOCK  5
#define TIMESTAMP_LENGTH       6
#define LINE_LENGTH(x)         ((x * 2) + (x - 1) + 2)

/*! Data Injection Structure */
struct data_inject
{
    FILE *in_log_ptr;
    uint32_t file_size;
    int32_t total_line;
    uint8_t sensor_data_size;
    uint32_t sensor_block_size;
};

/**
* @brief Function to initialize Data Injection structure
* @param[in] id         : Sensor
* @param[in] input      : Input Field Log
* @param[in] dinject    : Data Injection structure
* @param[in] bhy2       : Device reference
* @return API error codes
*/
int8_t dinject_init(uint8_t id, char *input, struct data_inject *dinject, struct bhy2_dev *bhy2);

/**
* @brief Function to get the Timestamp and read a data sample
* @param[in] dinject     : Data Injection structure
* @param[in] bhy2        : Device reference
* @return API error codes
*/
int8_t dinject_log_init(struct data_inject *dinject, struct bhy2_dev *bhy2);

/**
* @brief Function to inject the data
* @param[in] actual_len  : Size of the data to be injected
* @param[in] dinject     : Data Injection structure
* @param[in] bhy2        : Device reference
* @return API error codes
*/
int8_t dinject_inject_data(uint32_t *actual_len, struct data_inject *dinject, struct bhy2_dev *bhy2);

/**
* @brief Function to uninitialize the Data Injection structure
* @param[in] dinject    : Data Injection structure
* @param[in] bhy2        : Device reference
* @return API error codes
*/
int8_t dinject_deinit(struct data_inject *dinject, struct bhy2_dev *bhy2);

#endif /* BHY2CLI_DINJECT_H_ */
