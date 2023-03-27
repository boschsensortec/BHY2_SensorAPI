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
 * @file    dinject.c
 * @brief   Source file for the data injection functions for the command line utility
 *
 */

#include <stdio.h>
#include "parse.h"
#include "bhy2_parse.h"
#include "verbose.h"
#include "coines.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <sys/stat.h>
#include "dinject.h"

#define DI_PARSE_ERROR    -1
#define DI_PARSE_SUCCESS  1

uint32_t pos = 0;
uint8_t inject_log[MAX_SAMPLE_LENGTH] = { 0 };
uint16_t event_size = 0;
uint16_t len = 0;

/*******************************************************************************/
int dinject_hex_to_dec(char *hex, uint8_t len)
{
    int hex_val = 0, dec_val = 0, exponent = 0;

    for (int i = len - 1; i >= 0; --i)
    {
        if (hex[i] >= '0' && hex[i] <= '9')
        {
            hex_val = hex[i] - '0';
        }
        else if ((hex[i] >= 'A' && hex[i] <= 'F') || (hex[i] >= 'a' && hex[i] <= 'f'))
        {
            switch (hex[i])
            {
                case 'A':
                case 'a':
                    hex_val = 10;
                    break;
                case 'B':
                case 'b':
                    hex_val = 11;
                    break;
                case 'C':
                case 'c':
                    hex_val = 12;
                    break;
                case 'D':
                case 'd':
                    hex_val = 13;
                    break;
                case 'E':
                case 'e':
                    hex_val = 14;
                    break;
                case 'F':
                case 'f':
                    hex_val = 15;
                    break;
            }
        }

        dec_val = dec_val + hex_val * pow(16, exponent);
        ++exponent;
    }

    return dec_val;
}

int8_t dinject_parse_file(FILE *fp, size_t hex_len, size_t event_size, uint8_t int_stream[])
{
    char char_string[8];
    char single_char;
    int i = 0;
    size_t string_len = 0;
    size_t data_size = hex_len - 1;
    size_t num_elements = 1;

    while (event_size > 0)
    {
        single_char = fgetc(fp);

        if ((single_char == ' ') || (single_char == '\r') || (single_char == '\n'))
        {
            continue;
        }
        else
        {
            if (single_char != EOF)
            {
                char_string[0] = single_char;
                string_len = fread(&char_string[1], data_size, num_elements, fp);
                if (string_len != num_elements)
                {
                    return DI_PARSE_ERROR;
                }

                int_stream[i] = dinject_hex_to_dec(char_string, hex_len);
                memset(char_string, 0, sizeof(char_string));
                i++;
                event_size = event_size - 1;
            }
            else
            {
                break;
            }
        }
    }

    return DI_PARSE_SUCCESS;
}

/*******************************************************************************/

/**
* @brief Function to initialize Data Injection structure
*/
int8_t dinject_init(char *input, struct data_inject *dinject, struct bhy2_dev *bhy2)
{
    struct stat st;
    int8_t rslt = BHY2_OK;

    /*! Open the Field Log*/
#ifdef PC
    dinject->in_log_ptr = fopen(input, "rb");
#else
    dinject->in_log_ptr = fopen(input, "r");
#endif

    if (!dinject->in_log_ptr)
    {
        ERROR("Could not open file %s\r\n\r\n", input);

        return BHY2_E_INVALID_PARAM;
    }
    else
    {
        PRINT("Opened Log File %s \r\n\r\n", input);
    }

    /*! Compute the File size */
    stat((char*)input, &st);
    dinject->file_size = st.st_size;
    PRINT("File Size : %ld\r\n", dinject->file_size);

    return rslt;
}

/**
 * @brief Function to inject the data
*/
int8_t dinject_inject_data(int event_id, struct data_inject *dinject, struct bhy2_dev *bhy2)
{
    int8_t rslt;

    if (feof(dinject->in_log_ptr) || (event_id == EOF))
    {
        PRINT("Data Injection Completed\r\n");

        return DINJECT_SUCCESSFUL;
    }
    else
    {
        switch (event_id)
        {
            case ACCEL_INJECT_ID:
                event_size = ACCEL_INJECT_EVENT_SIZE;
                inject_log[len] = event_id;
                rslt =
                    dinject_parse_file(dinject->in_log_ptr, _8BIT_HEX_LEN, ACCEL_INJECT_EVENT_SIZE - 1,
                                       &inject_log[len + 1]);
                if (rslt != DI_PARSE_SUCCESS)
                {
                    ERROR("File Parsing failed \r\n");
                }

                len += event_size;
                break;

            case GYRO_INJECT_ID:
                event_size = GYRO_INJECT_EVENT_SIZE;
                inject_log[len] = event_id;
                rslt =
                    dinject_parse_file(dinject->in_log_ptr, _8BIT_HEX_LEN, GYRO_INJECT_EVENT_SIZE - 1,
                                       &inject_log[len + 1]);
                if (rslt != DI_PARSE_SUCCESS)
                {
                    ERROR("File Parsing failed \r\n");
                }

                len += event_size;
                break;

            case TIMESTAMP_LARGE_DELTA_WU_ID:
            case TIMESTAMP_LARGE_DELTA_NWU_ID:
                if (len != 0)
                {

                    rslt = bhy2_inject_data(inject_log, len, bhy2);
                    if (rslt == 0)
                    {
                        memset(inject_log, 0, sizeof(inject_log));
                    }

                    len = 0;
                }

                event_size = TIMESTAMP_LARGE_DELTA_EVENT_SIZE;
                inject_log[len] = event_id;
                rslt = dinject_parse_file(dinject->in_log_ptr,
                                          _8BIT_HEX_LEN,
                                          TIMESTAMP_LARGE_DELTA_EVENT_SIZE - 1,
                                          &inject_log[len + 1]);
                if (rslt != DI_PARSE_SUCCESS)
                {
                    ERROR("File Parsing failed \r\n");
                }

                len += event_size;
                break;

            case TIMESTAMP_SMALL_DELTA_WU_ID:
            case TIMESTAMP_SMALL_DELTA_NWU_ID:
                if (len != 0)
                {
                    rslt = bhy2_inject_data(inject_log, len, bhy2);
                    if (rslt == 0)
                    {
                        memset(inject_log, 0, sizeof(inject_log));
                    }

                    len = 0;
                }

                event_size = TIMESTAMP_SMALL_DELTA_EVENT_SIZE;
                inject_log[len] = event_id;
                rslt = dinject_parse_file(dinject->in_log_ptr,
                                          _8BIT_HEX_LEN,
                                          TIMESTAMP_SMALL_DELTA_EVENT_SIZE - 1,
                                          &inject_log[len + 1]);
                if (rslt != DI_PARSE_SUCCESS)
                {
                    ERROR("File Parsing failed \r\n");
                }

                len += event_size;
                break;

            case FULL_TIMESTAMP_WU_ID:
            case FULL_TIMESTAMP_NWU_ID:
                event_size = FULL_TIMESTAMP_EVENT_SIZE;
                inject_log[len] = event_id;
                rslt = dinject_parse_file(dinject->in_log_ptr,
                                          _8BIT_HEX_LEN,
                                          FULL_TIMESTAMP_EVENT_SIZE - 1,
                                          &inject_log[len + 1]);
                if (rslt != DI_PARSE_SUCCESS)
                {
                    ERROR("File Parsing failed \r\n");
                }

                rslt = bhy2_inject_data(inject_log, event_size, bhy2);
                if (rslt == 0)
                {
                    memset(inject_log, 0, sizeof(inject_log));
                }

                len = 0;
                break;

            case META_EVENT_WU_ID:
            case META_EVENT_NWU_ID:
                event_size = META_EVENT_SIZE;
                inject_log[len] = event_id;
                rslt =
                    dinject_parse_file(dinject->in_log_ptr, _8BIT_HEX_LEN, META_EVENT_SIZE - 1, &inject_log[len + 1]);
                if (rslt != DI_PARSE_SUCCESS)
                {
                    ERROR("File Parsing failed \r\n");
                }

                rslt = bhy2_inject_data(inject_log, event_size, bhy2);
                if (rslt == 0)
                {
                    memset(inject_log, 0, sizeof(inject_log));
                }

                len = 0;
                break;

            case DEBUT_EVENT_ID:
                event_size = DEBUG_EVENT_SIZE;
                inject_log[len] = event_id;
                rslt =
                    dinject_parse_file(dinject->in_log_ptr, _8BIT_HEX_LEN, DEBUG_EVENT_SIZE - 1, &inject_log[len + 1]);
                if (rslt != DI_PARSE_SUCCESS)
                {
                    ERROR("File Parsing failed \r\n");
                }

                rslt = bhy2_inject_data(inject_log, event_size, bhy2);
                if (rslt == 0)
                {
                    memset(inject_log, 0, sizeof(inject_log));
                }

                len = 0;
                break;

            default:
                pos = ftell(dinject->in_log_ptr);
                PRINT("\r\nError parsing 0x%X @ %u (0x%X)\r\n", event_id, pos, pos);

                return DINJECT_FAILED;
        }
    }

    return DINJECT_IN_PROGRESS;
}

/**
* @brief Function to uninitialize the Data Injection structure
*/
int8_t dinject_deinit(struct data_inject *dinject, struct bhy2_dev *bhy2)
{
    pos = 0;
    memset(inject_log, 0, sizeof(inject_log));

    /*! Close the Field Log file, if open */
    PRINT("Closing the files\r\n");

    if (dinject->in_log_ptr)
    {
        fclose(dinject->in_log_ptr);
    }

    /*! Reset the Data Injection Structure */
    memset(dinject, 0, sizeof(struct data_inject));

    return BHY2_OK;
}
