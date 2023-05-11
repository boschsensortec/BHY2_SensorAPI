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
 * @file    verbose.h
 *
 */

#ifndef _VERBOSE_H_
#define _VERBOSE_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include "coines.h"

#define VERBOSE_BUFFER_SIZE  256

void verbose(const char * __restrict__ _Format, ...);
void verbose_error(const char * __restrict__ _Format, ...);
void verbose_warning(const char * __restrict__ _Format, ...);
void verbose_info(const char * __restrict__ _Format, ...);

int8_t verb_help(void *ref);

int8_t verbose_callback(uint8_t argc, uint8_t *argv[], void *ref);

#define PRINT(format, ...)    verbose(format,##__VA_ARGS__)
#define INFO(format, ...)     verbose_info("[I]"format,##__VA_ARGS__)
#define PRINT_I(format, ...)  verbose_info(format,##__VA_ARGS__)
#define WARNING(format, ...)  verbose_warning("[W]"format,##__VA_ARGS__)
#define PRINT_W(format, ...)  verbose_warning(format,##__VA_ARGS__)
#define ERROR(format, ...)    verbose_error("[E]"format,##__VA_ARGS__)
#define PRINT_E(format, ...)  verbose_error(format,##__VA_ARGS__)

#define DATA(format, ...)     verbose("[D]"format,##__VA_ARGS__)
#define PRINT_D(format, ...)  verbose(format,##__VA_ARGS__)

#define HEX(format, ...)      verbose("[H]"format,##__VA_ARGS__)
#define PRINT_H(format, ...)  verbose(format,##__VA_ARGS__)

void verbose_write(uint8_t *buffer, uint16_t length);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* _VERBOSE_H_ */
