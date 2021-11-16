/**
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
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
 * @file    parse.c
 * @date    24 Mar 2020
 * @brief   Source file for the parse functions for the command line utility
 *
 */

#include <stdio.h>
#include "parse.h"
#include "bhy2_parse.h"

static uint8_t verbose;

/* #define PRINT(format, ...)   printf(format,##__VA_ARGS__) */
#define INFO(format, ...)    if (verbose >= 2) printf("[Info]"format,##__VA_ARGS__)
#define PRINT_I(format, ...) if (verbose >= 2) printf(format,##__VA_ARGS__)

/* #define WARNING(format, ...) if (verbose >= 1) printf("[Warning]"format,##__VA_ARGS__) */
/* #define PRINT_W(format, ...) if (verbose >= 1) printf(format,##__VA_ARGS__) */
#define ERROR(format, ...)   printf("[Error]"format,##__VA_ARGS__)

/* #define PRINT_E(format, ...) printf(format,##__VA_ARGS__) */

static void time_to_s_ns(uint64_t time_ticks, uint32_t *s, uint32_t *ns)
{
    uint64_t timestamp = time_ticks; /* Store the last timestamp */

    timestamp = timestamp * 15625; /* timestamp is now in nanoseconds */
    *s = (uint32_t)(timestamp / UINT64_C(1000000000));
    *ns = (uint32_t)(timestamp - ((*s) * UINT64_C(1000000000)));
}

/* BHY2_SYS_ID_META_EVENT, BHY2_SYS_ID_META_EVENT_WU */
void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    uint32_t s, ns;
    char *event_text;

    if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT)
    {
        event_text = "[META EVENT]";
    }
    else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU)
    {
        event_text = "[META EVENT WAKE UP]";
    }
    else
    {
        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns);

    struct parse_ref *parse_table = (struct parse_ref*)callback_ref;
    verbose = *(parse_table->verbose);

    switch (meta_event_type)
    {
        case BHY2_META_EVENT_FLUSH_COMPLETE:
            INFO("%s; T: %u.%09u; Flush complete for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
            INFO("%s; T: %u.%09u; Sample rate changed for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHY2_META_EVENT_POWER_MODE_CHANGED:
            INFO("%s; T: %u.%09u; Power mode changed for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHY2_META_EVENT_ALGORITHM_EVENTS:
            INFO("%s; T: %u.%09u; Algorithm event\r\n", event_text, s, ns);
            break;
        case BHY2_META_EVENT_SENSOR_STATUS:
            INFO("%s; T: %u.%09u; Accuracy for sensor id %u changed to %u\r\n", event_text, s, ns, byte1, byte2);
            if (parse_table)
            {
                parse_table->sensor[byte1].accuracy = byte2;
            }
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
            INFO("%s; T: %u.%09u; BSX event (do steps main)\r\n", event_text, s, ns);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
            INFO("%s; T: %u.%09u; BSX event (do steps calib)\r\n", event_text, s, ns);
            break;
        case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            INFO("%s; T: %u.%09u; BSX event (get output signal)\r\n", event_text, s, ns);
            break;
        case BHY2_META_EVENT_SENSOR_ERROR:
            INFO("%s; T: %u.%09u; Sensor id %u reported error 0x%02X\r\n", event_text, s, ns, byte1, byte2);
            break;
        case BHY2_META_EVENT_FIFO_OVERFLOW:
            INFO("%s; T: %u.%09u; FIFO overflow\r\n", event_text, s, ns);
            break;
        case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
            INFO("%s; T: %u.%09u; Dynamic range changed for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHY2_META_EVENT_FIFO_WATERMARK:
            INFO("%s; T: %u.%09u; FIFO watermark reached\r\n", event_text, s, ns);
            break;
        case BHY2_META_EVENT_INITIALIZED:
            INFO("%s; T: %u.%09u; Firmware initialized. Firmware version %u\r\n", event_text, s, ns,
                 ((uint16_t )byte2 << 8) | byte1);
            break;
        case BHY2_META_TRANSFER_CAUSE:
            INFO("%s; T: %u.%09u; Transfer cause for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHY2_META_EVENT_SENSOR_FRAMEWORK:
            INFO("%s; T: %u.%09u; Sensor framework event for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHY2_META_EVENT_RESET:
            INFO("%s; T: %u.%09u; Reset event\r\n", event_text, s, ns);
            break;
        case BHY2_META_EVENT_SPACER:
            break;
        default:
            INFO("%s; T: %u.%09u; Unknown meta event with id: %u\r\n", event_text, s, ns, meta_event_type);
            break;
    }
}

void parse_3axis_s16(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhy2_data_xyz data;
    uint32_t s, ns;

    if (callback_ref)
    {
        struct parse_ref *parse_table = (struct parse_ref*)callback_ref;
        verbose = *(parse_table->verbose);
        float scaling_factor;

        scaling_factor = parse_table->sensor[callback_info->sensor_id].scaling_factor;

        bhy2_parse_xyz(callback_info->data_ptr, &data);

        time_to_s_ns(*callback_info->time_stamp, &s, &ns);

        INFO("SID: %u; T: %u.%09u; x: %f, y: %f, z: %f; acc: %u\r\n",
             callback_info->sensor_id,
             s,
             ns,
             data.x * scaling_factor,
             data.y * scaling_factor,
             data.z * scaling_factor,
             parse_table->sensor[callback_info->sensor_id].accuracy);
    }
    else
    {
        ERROR("Null reference\r\r\n");
    }
}

void parse_euler(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhy2_data_orientation data;
    uint32_t s, ns;

    if (callback_ref)
    {
        struct parse_ref *parse_table = (struct parse_ref*)callback_ref;
        verbose = *(parse_table->verbose);
        float scaling_factor;

        scaling_factor = parse_table->sensor[callback_info->sensor_id].scaling_factor;

        bhy2_parse_orientation(callback_info->data_ptr, &data);

        time_to_s_ns(*callback_info->time_stamp, &s, &ns);

        INFO("SID: %u; T: %u.%09u; h: %f, p: %f, r: %f; acc: %u\r\n",
             callback_info->sensor_id,
             s,
             ns,
             data.heading * scaling_factor,
             data.pitch * scaling_factor,
             data.roll * scaling_factor,
             parse_table->sensor[callback_info->sensor_id].accuracy);
    }
    else
    {
        ERROR("Null reference\r\r\n");
    }
}

void parse_quaternion(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhy2_data_quaternion data;
    uint32_t s, ns;
    struct parse_ref *parse_table = (struct parse_ref*)callback_ref;

    verbose = *(parse_table->verbose);

    bhy2_parse_quaternion(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns);

    INFO("SID: %u; T: %u.%09u; x: %f, y: %f, z: %f, w: %f; acc: %f\r\n",
         callback_info->sensor_id,
         s,
         ns,
         data.x / 16384.0f,
         data.y / 16384.0f,
         data.z / 16384.0f,
         data.w / 16384.0f,
         ((data.accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);
}

void parse_s16_as_float(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    int16_t data;
    uint32_t s, ns;
    struct parse_ref *parse_table = (struct parse_ref*)callback_ref;

    verbose = *(parse_table->verbose);
    float scaling_factor = 0.0f;
    if (parse_table)
    {
        scaling_factor = parse_table->sensor[callback_info->sensor_id].scaling_factor;
    }

    data = BHY2_LE2S16(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns);

    INFO("SID: %u; T: %u.%09u; %f\r\n", callback_info->sensor_id, s, ns, data * scaling_factor);
}

void parse_scalar_u32(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t data;
    uint32_t s, ns;
    struct parse_ref *parse_table = (struct parse_ref*)callback_ref;

    verbose = *(parse_table->verbose);

    data = BHY2_LE2U32(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns);

    INFO("SID: %u; T: %u.%09u; %u\r\n", callback_info->sensor_id, s, ns, data);
}

void parse_scalar_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    struct parse_ref *parse_table = (struct parse_ref*)callback_ref;

    verbose = *(parse_table->verbose);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns);

    INFO("SID: %u; T: %u.%09u;\r\n", callback_info->sensor_id, s, ns);
}

void parse_activity(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint16_t activity;
    uint32_t s, ns;
    struct parse_ref *parse_table = (struct parse_ref*)callback_ref;

    verbose = *(parse_table->verbose);

    activity = BHY2_LE2U16(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns);

    INFO("SID: %u; T: %u.%09u; ", callback_info->sensor_id, s, ns);

    if (activity & BHY2_STILL_ACTIVITY_ENDED)
    {
        PRINT_I(" Still activity ended,");
    }

    if (activity & BHY2_WALKING_ACTIVITY_ENDED)
    {
        PRINT_I(" Walking activity ended,");
    }

    if (activity & BHY2_RUNNING_ACTIVITY_ENDED)
    {
        PRINT_I(" Running activity ended,");
    }

    if (activity & BHY2_ON_BICYCLE_ACTIVITY_ENDED)
    {
        PRINT_I(" On bicycle activity ended,");
    }

    if (activity & BHY2_IN_VEHICLE_ACTIVITY_ENDED)
    {
        PRINT_I(" In vehicle ended,");
    }

    if (activity & BHY2_TILTING_ACTIVITY_ENDED)
    {
        PRINT_I(" Tilting activity ended,");
    }

    if (activity & BHY2_STILL_ACTIVITY_STARTED)
    {
        PRINT_I(" Still activity started,");
    }

    if (activity & BHY2_WALKING_ACTIVITY_STARTED)
    {
        PRINT_I(" Walking activity started,");
    }

    if (activity & BHY2_RUNNING_ACTIVITY_STARTED)
    {
        PRINT_I(" Running activity started,");
    }

    if (activity & BHY2_ON_BICYCLE_ACTIVITY_STARTED)
    {
        PRINT_I(" On bicycle activity started,");
    }

    if (activity & BHY2_IN_VEHICLE_ACTIVITY_STARTED)
    {
        PRINT_I(" In vehicle activity started,");
    }

    if (activity & BHY2_TILTING_ACTIVITY_STARTED)
    {
        PRINT_I(" Tilting activity started,");
    }

    PRINT_I("\r\n");
}

void parse_u16_as_float(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint16_t data;
    uint32_t s, ns;
    struct parse_ref *parse_table = (struct parse_ref*)callback_ref;

    verbose = *(parse_table->verbose);
    float scaling_factor = 0.0f;
    if (parse_table)
    {
        scaling_factor = parse_table->sensor[callback_info->sensor_id].scaling_factor;
    }

    data = BHY2_LE2U16(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns);

    INFO("SID: %u; T: %u.%09u; %f\r\n", callback_info->sensor_id, s, ns, data * scaling_factor);
}

void parse_u24_as_float(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t data;
    uint32_t s, ns;
    struct parse_ref *parse_table = (struct parse_ref*)callback_ref;

    verbose = *(parse_table->verbose);
    float scaling_factor = 0.0f;
    if (parse_table)
    {
        scaling_factor = parse_table->sensor[callback_info->sensor_id].scaling_factor;
    }

    data = BHY2_LE2U24(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns);

    INFO("SID: %u; T: %u.%09u; %f\r\n", callback_info->sensor_id, s, ns, (float )data * scaling_factor);
}

void parse_proximity(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    char *text;
    uint32_t s, ns;
    struct parse_ref *parse_table = (struct parse_ref*)callback_ref;

    verbose = *(parse_table->verbose);

    text = callback_info->data_ptr[0] ? "near" : "far";

    time_to_s_ns(*callback_info->time_stamp, &s, &ns);

    INFO("SID: %u; T: %u.%09u; %s\r\n", callback_info->sensor_id, s, ns, text);
}

void parse_scalar_u8(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint8_t data;
    uint32_t s, ns;
    struct parse_ref *parse_table = (struct parse_ref*)callback_ref;

    verbose = *(parse_table->verbose);

    data = callback_info->data_ptr[0];

    time_to_s_ns(*callback_info->time_stamp, &s, &ns);

    INFO("SID: %u; T: %u.%09u; %u\r\n", callback_info->sensor_id, s, ns, data);
}

void parse_generic(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    struct parse_ref *parse_table = (struct parse_ref*)callback_ref;

    verbose = *(parse_table->verbose);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns);

    INFO("SID: %u; T: %u.%09u; ", callback_info->sensor_id, s, ns);
    for (uint8_t i = 0; i < (callback_info->data_size - 1); i++)
    {
        PRINT_I("%X ", callback_info->data_ptr[i]);
    }
    PRINT_I("\r\n");
}

void parse_device_ori(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    char *ori;
    uint32_t s, ns;
    struct parse_ref *parse_table = (struct parse_ref*)callback_ref;

    verbose = *(parse_table->verbose);

    switch (callback_info->data_ptr[0])
    {
        case 0:
            ori = "Portrait upright";
            break;
        case 1:
            ori = "Landscape left";
            break;
        case 2:
            ori = "Portrait upside down";
            break;
        case 3:
            ori = "Landscape right";
            break;
        default:
            ori = "Unknown orientation";
            break;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns);

    INFO("SID: %u; T: %u.%09u; %s\r\n", callback_info->sensor_id, s, ns, ori);
}

void parse_gps(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    struct parse_ref *parse_table = (struct parse_ref*)callback_ref;

    verbose = *(parse_table->verbose);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns);

    callback_info->data_ptr[callback_info->data_size - 2] = '\0';

    INFO("[DEBUG MSG]; T: %u.%09u; data: %s\r\n", s, ns, callback_info->data_ptr);
}

void parse_debug_message(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    struct parse_ref *parse_table = (struct parse_ref*)callback_ref;

    verbose = *(parse_table->verbose);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns);

    INFO("[DEBUG MSG]; T: %u.%09u; flag: 0x%x; data: %s\r\n",
         s,
         ns,
         callback_info->data_ptr[0],
         &callback_info->data_ptr[1]);
}
