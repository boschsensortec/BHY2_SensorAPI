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
 * @file       decompressor.c
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "logbin.h"

#define LUT_STRING_SIZE           (40)
#define EVENT_BUFFER_SIZE         (65535)

#define LUT_LE2U16(x)             ((uint16_t)((x)[0] | (x)[1] << 8))
#define LUT_LE2S16(x)             ((int16_t)LUT_LE2U16(x))
#define LUT_LE2U24(x)             ((uint32_t)((x)[0] | (uint32_t)(x)[1] << 8 | (uint32_t)(x)[2] << 16))
#define LUT_LE2S24(x)             ((int32_t)(LUT_LE2U24(x) << 8) >> 8)
#define LUT_LE2U32(x)             ((uint32_t)((x)[0] | (uint32_t)(x)[1] << 8 | (uint32_t)(x)[2] << 16 | \
                                              (uint32_t)(x)[3] << 24))
#define LUT_LE2S32(x)             ((int32_t)LUT_LE2U32(x))
#define LUT_LE2U40(x)             (LUT_LE2U32(x) | (uint64_t)(x)[4] << 32)
#define LUT_LE2U64(x)             (LUT_LE2U32(x) | (uint64_t)LUT_LE2U32(&(x)[4]) << 32)

static bool label_available = false;
static bool time_ns_available = false;
static uint8_t event_buffer[EVENT_BUFFER_SIZE] = { 0 };

/*#define DEBUG_PRINT(format, ...)    printf(format,##__VA_ARGS__) */
#define DEBUG_PRINT(format, ...)

/* Look up table that contains meta information for parsing the log file */
static struct
{
    char name[LUT_STRING_SIZE];
    uint8_t event_size;
    char parse_format[LUT_STRING_SIZE];
    char axis_names[LUT_STRING_SIZE];
    float scaling;
    bool meta_available; /* Meta information is available for parsing */
    bool data_available; /* Data is available, in order to write the header */
    bool new_data; /* New data is available to be printed into the output file */
    uint8_t *buffer_ptr;
}
meta_lut[LOGBIN_META_ID_START];

/* Get the version */
static void get_version(uint8_t *version, uint8_t n, FILE *input)
{
    char str[10] = { 0 };
    char c;
    uint8_t i = 0;
    char delimiter[] = ".";
    char *str_tok;

    do
    {
        c = fgetc(input);

        if ((c == EOF) || (c == '\n'))
        {
            break;
        }

        str[i] = c;
        i++;
    } while (c != EOF);

    i = 0;
    str_tok = strtok(str, delimiter);
    while ((str_tok != NULL) && (i < n))
    {
        version[i] = atoi(str_tok);
        str_tok = strtok(NULL, delimiter);
        i++;
    }
}

/* Get a parameter (token) of the meta information */
static bool get_param(char *str, FILE *input)
{
    char c;
    uint8_t i = 0;

    do
    {
        c = fgetc(input);

        if ((c == EOF) || (c == '\n'))
        {
            return false;
        }

        if (c == ':')
        {
            str[i] = '\0';

            return true;
        }

        if (c == ' ')
        {
            if (i == 0)
            {
                continue;
            }
        }

        str[i] = c;
        i++;
    } while (c != EOF);

    return false;
}

/* Parse a line of meta information */
static bool read_meta_line(uint8_t *sensor_id,
                           char *name,
                           uint8_t *event_size,
                           char *parse_format,
                           char *axis_names,
                           float *scaling,
                           FILE *input)
{
    char num[LUT_STRING_SIZE] = { 0 };

    if (get_param(num, input))
    {
        *sensor_id = atoi(num);
    }
    else
    {
        return false;
    }

    get_param(name, input);

    get_param(num, input);
    *event_size = atoi(num);

    get_param(parse_format, input);

    get_param(axis_names, input);

    get_param(num, input);
    *scaling = atof(num);

    return true;
}

/* Update the meta data look up table */
static void update_meta_lut(FILE *input)
{
    uint8_t sensor_id = 0, event_size = 0;
    char name[LUT_STRING_SIZE] = { 0 }, parse_format[LUT_STRING_SIZE] = { 0 }, axis_names[LUT_STRING_SIZE] = { 0 };
    float scaling = 0.0f;

    while (read_meta_line(&sensor_id, name, &event_size, parse_format, axis_names, &scaling, input))
    {
        if (sensor_id < LOGBIN_META_ID_START)
        {
            meta_lut[sensor_id].event_size = event_size;
            meta_lut[sensor_id].scaling = scaling;
            strcpy(meta_lut[sensor_id].name, name);
            strcpy(meta_lut[sensor_id].parse_format, parse_format);
            strcpy(meta_lut[sensor_id].axis_names, axis_names);
            meta_lut[sensor_id].meta_available = true;
        }
    }
}

/* Update the meta data look up table to know which data is actually in the log */
static void update_data_available(FILE *input)
{
    int16_t c;
    uint32_t pos_init = ftell(input), pos;

    do
    {
        c = fgetc(input);
        pos = ftell(input);

        if (c == EOF)
        {
            fseek(input, pos_init, SEEK_SET);

            return;
        }
        else if (c == LOGBIN_META_ID_TIME_NS)
        {
            fseek(input, pos + LOGBIN_TIME_NS_SIZE, SEEK_SET);
            time_ns_available = true;
        }
        else if (c == LOGBIN_META_ID_LABEL)
        {
            fseek(input, pos + LOGBIN_LABEL_SIZE, SEEK_SET);
            label_available = true;
        }
        else if (c < LOGBIN_META_ID_START)
        {
            meta_lut[c].data_available = true;
            fseek(input, pos + meta_lut[c].event_size, SEEK_SET);
        }
        else
        {
            printf("\tError parsing 0x%X @ %u (0x%X)\n", c, pos, pos);
            exit(1);
        }
    } while (c != EOF);

    fseek(input, pos_init, SEEK_SET);
}

/* Print the header for the output file */
static void print_output_header(FILE *output)
{
    char tmp_axis_names[LUT_STRING_SIZE];
    char *str_tok = NULL;
    char *delimiter = ",";

    if (time_ns_available)
    {
        fprintf(output, "Time(s.ns), ");
        DEBUG_PRINT("Time(s.ns), ");
    }

    for (uint8_t i = 0; i < LOGBIN_META_ID_START; i++)
    {
        if (meta_lut[i].meta_available && meta_lut[i].data_available)
        {
            strcpy(tmp_axis_names, meta_lut[i].axis_names);
            str_tok = strtok(tmp_axis_names, delimiter);
            while (str_tok != NULL)
            {
                fprintf(output, "%s.%s, ", meta_lut[i].name, str_tok);
                DEBUG_PRINT("%s.%s, ", meta_lut[i].name, str_tok);
                str_tok = strtok(NULL, delimiter);
            }
        }
    }

    if (label_available)
    {
        fprintf(output, "Label, ");
        DEBUG_PRINT("Label, ");
    }
}

/* Read and convert time to seconds and nanoseconds */
static void get_time_s_ns(uint32_t *s, uint32_t *ns, FILE *input)
{
    union
    {
        uint64_t data;
        uint8_t byte[LOGBIN_TIME_NS_SIZE];
    }
    time_ns;

    fread(time_ns.byte, 1, LOGBIN_TIME_NS_SIZE, input);

    *s = (uint32_t)(time_ns.data / UINT64_C(1000000000));
    *ns = (uint32_t)(time_ns.data - ((*s) * UINT64_C(1000000000)));
}

/* Read an event and store a reference into the look up table */
static uint16_t read_event(uint8_t id, uint8_t *buffer, FILE *input)
{
    if (meta_lut[id].event_size)
    {
        fread(buffer, 1, meta_lut[id].event_size, input);
        meta_lut[id].buffer_ptr = buffer;
    }

    meta_lut[id].new_data = true;

    return meta_lut[id].event_size;
}

/* Parse an event and print it in the output file */
static void parse_and_print_event(uint8_t id, FILE *output)
{
    char tmp_parse_format[LUT_STRING_SIZE];
    char *str_tok = NULL;
    char *delimiter = ",";
    uint8_t tmp_u8 = 0;
    uint16_t tmp_u16 = 0;
    uint32_t tmp_u32 = 0;
    int8_t tmp_s8 = 0;
    int16_t tmp_s16 = 0;
    int32_t tmp_s32 = 0;
    uint8_t tmp_data_c = 0;
    char *tmp_str = NULL;
    uint8_t idx = 0;

    union
    {
        uint32_t data_u32;
        float data_float;
    }
    u32_to_float;

    if (meta_lut[id].event_size == 0)
    {
        if (meta_lut[id].new_data)
        {
            fprintf(output, " 1,");
            DEBUG_PRINT(" 1,");
        }
        else
        {
            fprintf(output, " ,");
            DEBUG_PRINT(" ,");
        }

        meta_lut[id].new_data = false;
        meta_lut[id].buffer_ptr = NULL;

        return;
    }

    strcpy(tmp_parse_format, meta_lut[id].parse_format);
    str_tok = strtok(tmp_parse_format, delimiter);

    if (meta_lut[id].buffer_ptr && meta_lut[id].new_data)
    {
        while ((str_tok != NULL) && (idx < meta_lut[id].event_size))
        {
            if (strcmp(str_tok, "u8") == 0)
            {
                tmp_u8 = meta_lut[id].buffer_ptr[idx];
                idx += 1;

                if (meta_lut[id].scaling > 0.0f)
                {
                    fprintf(output, " %6.4f,", (float) tmp_u8 * meta_lut[id].scaling);
                    DEBUG_PRINT(" %6.4f,", (float) tmp_u8 * meta_lut[id].scaling);
                }
                else
                {
                    fprintf(output, " %u,", tmp_u8);
                    DEBUG_PRINT(" %u,", tmp_u8);
                }
            }
            else if (strcmp(str_tok, "u16") == 0)
            {
                tmp_u16 = LUT_LE2U16(&meta_lut[id].buffer_ptr[idx]);
                idx += 2;

                if (meta_lut[id].scaling > 0.0f)
                {
                    fprintf(output, " %6.4f,", (float) tmp_u16 * meta_lut[id].scaling);
                    DEBUG_PRINT(" %6.4f,", (float) tmp_u16 * meta_lut[id].scaling);
                }
                else
                {
                    fprintf(output, " %u,", tmp_u16);
                    DEBUG_PRINT(" %u,", tmp_u16);
                }
            }
            else if (strcmp(str_tok, "u24") == 0)
            {
                tmp_u32 = LUT_LE2U24(&meta_lut[id].buffer_ptr[idx]);
                idx += 3;

                if (meta_lut[id].scaling > 0.0f)
                {
                    fprintf(output, " %6.4f,", (float) tmp_u32 * meta_lut[id].scaling);
                    DEBUG_PRINT(" %6.4f,", (float) tmp_u32 * meta_lut[id].scaling);
                }
                else
                {
                    fprintf(output, " %u,", tmp_u32);
                    DEBUG_PRINT(" %u,", tmp_u32);
                }
            }
            else if (strcmp(str_tok, "u32") == 0)
            {
                tmp_u32 = LUT_LE2U32(&meta_lut[id].buffer_ptr[idx]);
                idx += 4;

                if (meta_lut[id].scaling > 0.0f)
                {
                    fprintf(output, " %6.4f,", (float) tmp_u32 * meta_lut[id].scaling);
                    DEBUG_PRINT(" %6.4f,", (float) tmp_u32 * meta_lut[id].scaling);
                }
                else
                {
                    fprintf(output, " %u,", tmp_u32);
                    DEBUG_PRINT(" %u,", tmp_u32);
                }
            }
            else if (strcmp(str_tok, "s8") == 0)
            {
                tmp_s8 = (int8_t) meta_lut[id].buffer_ptr[idx];
                idx += 1;

                if (meta_lut[id].scaling > 0.0f)
                {
                    fprintf(output, " %6.4f,", (float) tmp_s8 * meta_lut[id].scaling);
                    DEBUG_PRINT(" %6.4f,", (float) tmp_s8 * meta_lut[id].scaling);
                }
                else
                {
                    fprintf(output, " %d,", tmp_s8);
                    DEBUG_PRINT(" %d,", tmp_s8);
                }
            }
            else if (strcmp(str_tok, "s16") == 0)
            {
                tmp_s16 = LUT_LE2S16(&meta_lut[id].buffer_ptr[idx]);
                idx += 2;

                if (meta_lut[id].scaling > 0.0f)
                {
                    fprintf(output, " %6.4f,", (float) tmp_s16 * meta_lut[id].scaling);
                    DEBUG_PRINT(" %6.4f,", (float) tmp_s16 * meta_lut[id].scaling);
                }
                else
                {
                    fprintf(output, " %d,", tmp_s16);
                    DEBUG_PRINT(" %d,", tmp_s16);
                }
            }
            else if (strcmp(str_tok, "s24") == 0)
            {
                tmp_s32 = LUT_LE2S24(&meta_lut[id].buffer_ptr[idx]);
                idx += 3;

                if (meta_lut[id].scaling > 0.0f)
                {
                    fprintf(output, " %6.4f,", (float) tmp_s32 * meta_lut[id].scaling);
                    DEBUG_PRINT(" %6.4f,", (float) tmp_s32 * meta_lut[id].scaling);
                }
                else
                {
                    fprintf(output, " %d,", tmp_s32);
                    DEBUG_PRINT(" %d,", tmp_s32);
                }
            }
            else if (strcmp(str_tok, "s32") == 0)
            {
                tmp_s32 = LUT_LE2S32(&meta_lut[id].buffer_ptr[idx]);
                idx += 4;

                if (meta_lut[id].scaling > 0.0f)
                {
                    fprintf(output, " %6.4f,", (float) tmp_s32 * meta_lut[id].scaling);
                    DEBUG_PRINT(" %6.4f,", (float) tmp_s32 * meta_lut[id].scaling);
                }
                else
                {
                    fprintf(output, " %d,", tmp_s32);
                    DEBUG_PRINT(" %d,", tmp_s32);
                }
            }
            else if (strcmp(str_tok, "c") == 0)
            {
                tmp_data_c = meta_lut[id].buffer_ptr[idx];
                idx += 1;

                fprintf(output, " %c,", tmp_data_c);
                DEBUG_PRINT(" %c,", tmp_data_c);
            }
            else if (strcmp(str_tok, "f") == 0)
            {
                /* Float values have to be read as unsigned and then interpreted as float */
                u32_to_float.data_u32 = LUT_LE2U32(&meta_lut[id].buffer_ptr[idx]);
                idx += 4;

                /* The binary data has to be interpreted as a float */
                if (meta_lut[id].scaling > 0.0f)
                {
                    fprintf(output, " %6.8f,", (float) u32_to_float.data_float * meta_lut[id].scaling);
                    DEBUG_PRINT(" %6.8f,", (float) u32_to_float.data_float * meta_lut[id].scaling);
                }
                else
                {
                    fprintf(output, " %6.8f,", u32_to_float.data_float);
                    DEBUG_PRINT(" %6.8f,", u32_to_float.data_float);
                }
            }
            else if (strcmp(str_tok, "st") == 0)
            {
                tmp_str = (char *)&meta_lut[id].buffer_ptr[idx];
                idx = meta_lut[id].event_size; /* A string has to be the last or only parameter */

                fprintf(output, " %s,", tmp_str);
                DEBUG_PRINT(" %s,", tmp_str);
            }

            str_tok = strtok(NULL, delimiter);
        }
    }
    else
    {
        while (str_tok != NULL)
        {
            if (strcmp(str_tok, "u8") == 0)
            {
                fprintf(output, " ,");
                DEBUG_PRINT(" ,");
            }
            else if (strcmp(str_tok, "u16") == 0)
            {
                fprintf(output, " ,");
                DEBUG_PRINT(" ,");
            }
            else if (strcmp(str_tok, "u24") == 0)
            {
                fprintf(output, " ,");
                DEBUG_PRINT(" ,");
            }
            else if (strcmp(str_tok, "u32") == 0)
            {
                fprintf(output, " ,");
                DEBUG_PRINT(" ,");
            }
            else if (strcmp(str_tok, "s8") == 0)
            {
                fprintf(output, " ,");
                DEBUG_PRINT(" ,");
            }
            else if (strcmp(str_tok, "s16") == 0)
            {
                fprintf(output, " ,");
                DEBUG_PRINT(" ,");
            }
            else if (strcmp(str_tok, "s32") == 0)
            {
                fprintf(output, " ,");
                DEBUG_PRINT(" ,");
            }
            else if (strcmp(str_tok, "c") == 0)
            {
                fprintf(output, " ,");
                DEBUG_PRINT(" ,");
            }
            else if (strcmp(str_tok, "f") == 0)
            {
                fprintf(output, " ,");
                DEBUG_PRINT(" ,");
            }

            str_tok = strtok(NULL, delimiter);
        }
    }

    meta_lut[id].new_data = false;
    meta_lut[id].buffer_ptr = NULL;
}

/* Parse and print thw whole line */
static void parse_and_print_line(FILE *output)
{
    for (uint8_t i = 0; i < LOGBIN_META_ID_START; i++)
    {
        if (meta_lut[i].meta_available && meta_lut[i].data_available)
        {
            parse_and_print_event(i, output);
        }
    }
}

/* Parse and print the whole data into the file */
static void print_data_to_file(FILE *input, FILE *output)
{
    uint32_t s = 0, ns = 0;
    int16_t c;
    uint32_t pos;
    char label_str[LOGBIN_LABEL_SIZE + 1] = { 0 };
    bool new_label = false;
    bool new_data = false;
    uint16_t buff_idx = 0;

    memset(event_buffer, 0, sizeof(event_buffer));

    do
    {
        c = fgetc(input);
        pos = ftell(input);

        if (c == EOF)
        {
            if (new_data)
            {
                parse_and_print_line(output);
                new_data = false;
            }

            return;
        }
        else if (c == LOGBIN_META_ID_TIME_NS)
        {
            /* Print the label on the before timestamp on the
             * previous line */
            if (new_data)
            {
                parse_and_print_line(output);
                new_data = false;
            }

            if (label_available)
            {
                if (new_label)
                {
                    fprintf(output, " %s,", label_str);
                    DEBUG_PRINT(" %s,", label_str);
                    memset(label_str, 0, sizeof(label_str));
                    new_label = false;
                }
                else
                {
                    fprintf(output, " ,");
                    DEBUG_PRINT(" ,");
                }
            }

            get_time_s_ns(&s, &ns, input);
            fprintf(output, "\n%u.%09u,", s, ns);
            DEBUG_PRINT("\n%u.%09u,", s, ns);
        }
        else if (c == LOGBIN_META_ID_LABEL)
        {
            fread(label_str, 1, LOGBIN_LABEL_SIZE, input);
            new_label = true;
        }
        else if (c < LOGBIN_META_ID_START)
        {
            if (meta_lut[c].meta_available)
            {
                new_data = true;
                buff_idx += read_event(c, &event_buffer[buff_idx], input);
            }
        }
        else
        {
            printf("\tError parsing 0x%X @ %u (0x%X)\n", c, pos, pos);
            exit(1);
        }
    } while (c != EOF);
}

/* Parse the whole input file */
static void parse_file(FILE *input, FILE *output)
{
    uint8_t version[2] = { 0 };

    get_version(version, 2, input);
    printf("\tMeta version %u.%u\n", version[0], version[1]);

    printf("\tLoading meta data\n");
    update_meta_lut(input);

    for (uint8_t i = 0; i < LOGBIN_META_ID_START; i++)
    {
        if (meta_lut[i].meta_available)
        {
            DEBUG_PRINT("\t%u: %s: %u: %s: %s: %f\n",
                        i,
                        meta_lut[i].name,
                        meta_lut[i].event_size,
                        meta_lut[i].parse_format,
                        meta_lut[i].axis_names,
                        meta_lut[i].scaling);
        }
    }

    printf("\tScanning available data frames\n");
    update_data_available(input);

    for (uint8_t i = 0; i < LOGBIN_META_ID_START; i++)
    {
        if (meta_lut[i].meta_available && meta_lut[i].data_available)
        {
            DEBUG_PRINT("\t%u: %s: %u: %s: %s: %f\n",
                        i,
                        meta_lut[i].name,
                        meta_lut[i].event_size,
                        meta_lut[i].parse_format,
                        meta_lut[i].axis_names,
                        meta_lut[i].scaling);
        }
    }

    DEBUG_PRINT("\n");
    printf("\tWriting the output file header\n");
    print_output_header(output);

    DEBUG_PRINT("\n");
    printf("\tWriting the output file data\n");
    print_data_to_file(input, output);
}

/* This function takes multiple input files, and provides them for
 * decompressing.
 */
int main(int argc, char *argv[])
{
    FILE *input_file, *output_file;
    char output_file_name[256]; /* Limit file name to 256 characters */

    printf("Utility to decompress binary log files\n");
    if (argc == 1)
    {
        printf("Pass one or more sensor data binary files as arguments\n");
        exit(-1);
    }
    else
    {
        for (int i = 1; i < argc; i++)
        {
            input_file = fopen(argv[i], "rb");
            if (input_file)
            {
                sprintf(output_file_name, "%s" ".csv", (char*) argv[i]);
                printf("Decompressing binary file to %s\n", output_file_name);
                output_file = fopen(output_file_name, "w");
                if (output_file)
                {
                    memset(meta_lut, 0, sizeof(meta_lut));
                    parse_file(input_file, output_file);
                    fflush(output_file);
                    fclose(output_file);
                }
                else
                {
                    printf("Could not create %s\n", output_file_name);
                }
            }
            else
            {
                printf("Could not open %s.\n", argv[i]);
            }

            fclose(input_file);
        }
    }
}
