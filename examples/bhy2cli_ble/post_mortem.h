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
 * @file    post_mortem.h
 * @brief   Header file for post mortem report retrieval
 **
 */

#ifndef _POST_MORTEM_H_
#define _POST_MORTEM_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <bhy2.h>

#define STACK_SIZE            4096

#define PM_DEBUG              0

#define PM_PARSE_FAILED       -1
#define PM_PARSE_SUCCESS      0

#define PM_LOG_FAILED         -1
#define PM_LOG_SUCCESS        0

/*! Error Register Values*/
/*! To be moved to bhy2_defs.h in future */
#define PM_DATA_AVAILABLE     0x44      /*Unhandled Interrupt Error/Exception/Post Mortem Data Available*/
#define WATCH_DOG_RESET       0x19      /*Unexpected Watchdog Reset*/
#define FATAL_FIRMWARE_ERROR  0x1B      /*Fatal Firmware Error*/

typedef struct bhy2_post_mortem
{
    uint16_t code; /* 0x00-0x01 */
    uint16_t length; /* 0x02-0x03 */
    uint32_t reg[32]; /* 0x04-0x83 */
    uint8_t valid; /* 0x84 */
    uint8_t flags; /* 0x85 */
    uint16_t stackSize; /* 0x86 */
    uint32_t stackStart; /* 0x88 */
    uint32_t eret; /* 0x8c */
    uint32_t erbta; /* 0x90 */
    uint32_t erstatus; /* 0x94 */
    uint32_t ecr; /* 0x98 */
    uint32_t efa; /* 0x9c */
    uint32_t diagnostic; /* 0xa0 */
    uint32_t icause; /* 0xa4 */
    uint8_t debugValue; /* 0xa8 */
    uint8_t debugState; /* 0xa9 */
    uint8_t errorReason; /* 0xaa */
    uint8_t interruptState; /* 0xab */
    uint8_t hostIntCtrl; /* 0xac */
    uint8_t resetReason; /* 0xad */
    uint8_t hostResetCount; /* 0xae */
    uint8_t errorReport; /* 0xaf */
    uint32_t mpu_ecr; /* 0xb0 */
    uint32_t user_sp; /* 0xb4 */
    uint8_t reserved[12]; /* 0xb8 */
    uint32_t stackCrc; /* 0xc4 */
    uint32_t crc; /* 0xc8 */
    uint8_t stackDump[STACK_SIZE];
} bhy2_post_mortem;

/**
* @brief Function to get the Post Mortem data
* @param[in] pminfo     : Post Mortem Structure
* @param[in] bhy2       : Device reference
* @return API error codes
*/
int8_t get_post_mortem_data(struct bhy2_post_mortem *pminfo, struct bhy2_dev *bhy2);

/**
* @brief Function to log the Post Mortem data
* @param[in] pmfilename : Log Filename
* @param[in] pminfo     : Post Mortem Structure
* @param[in] pmlen      : Size of the data to be logged
* @return API error codes
*/
int8_t log_post_mortem_data(const char *pmfilename, struct bhy2_post_mortem *pminfo, uint32_t pmlen);

/**
* @brief Function to read the Post Mortem data from the log
* @param[in] pmfilename : Log Filename
* @param[in] pminfo     : Post Mortem Structure
* @return API error codes
*/
int8_t read_post_mortem_data_from_log(const char *pmfilename, struct bhy2_post_mortem *pminfo);

#if PM_DEBUG

/**
* @brief Function to print the Post Mortem data
* @param[in] pminfo     : Post Mortem Structure
* @return API error codes
*/
int8_t print_post_mortem_data(struct bhy2_post_mortem *pminfo);

#endif

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* _POST_MORTEM_H_ */
