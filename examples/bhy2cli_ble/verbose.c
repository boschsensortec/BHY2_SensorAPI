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

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "coines.h"
#include "verbose.h"
#include "cli.h"

static uint8_t verb_lvl = 0;
#ifndef PC
static uint8_t verb_buff[VERBOSE_BUFFER_SIZE] = { 0 };
#endif

int8_t verb_help(void *ref)
{
    PRINT("  -v OR verb <verbose level>\r\n");
    PRINT("    \t= Set the verbose level. 0 Error, 1 Warning, 2 Infos\r\n");

    return CLI_OK;
}

int8_t verbose_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    verb_lvl = (uint8_t) (argv[1][0] - '0');
    if (verb_lvl > 2)
    {
        verb_lvl = 2;
    }

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    PRINT("Setting verbose to %u\r\n", verb_lvl);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

static void verb_out(uint8_t verb_ref, const char * __restrict__ format, va_list args)
{
    if (verb_lvl >= verb_ref)
    {
#ifdef PC
        vfprintf(stdout, format, args);
#else
        vsprintf((char *)verb_buff, format, args);
        verbose_write(verb_buff, strlen((char *)verb_buff));
#endif
    }
}

void verbose(const char * __restrict__ format, ...)
{
    va_list args;

    va_start(args, format);
    verb_out(0, format, args);
    va_end(args);
}

void verbose_error(const char * __restrict__ format, ...)
{
    va_list args;

    va_start(args, format);
    verb_out(0, format, args);
    va_end(args);
}

void verbose_warning(const char * __restrict__ format, ...)
{
    va_list args;

    va_start(args, format);
    verb_out(1, format, args);
    va_end(args);
}

void verbose_info(const char * __restrict__ format, ...)
{
    va_list args;

    va_start(args, format);
    verb_out(2, format, args);
    va_end(args);
}
