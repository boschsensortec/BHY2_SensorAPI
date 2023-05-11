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
 * @file    parse.c
 * @brief   Source file for the parse functions for the command line utility
 *
 */

#include <stdio.h>
#include "parse.h"
#include "bhy2_parse.h"
#include "verbose.h"
#include "coines.h"

static void time_to_s_ns(uint64_t time_ticks, uint32_t *s, uint32_t *ns, uint64_t *tns)
{
    *tns = time_ticks * 15625; /* timestamp is now in nanoseconds */
    *s = (uint32_t)(*tns / UINT64_C(1000000000));
    *ns = (uint32_t)(*tns - ((*s) * UINT64_C(1000000000)));
}

static void log_data(uint8_t sid, uint64_t tns, uint8_t event_size, uint8_t *event_payload, struct logbin_dev *logdev)
{
    if (logdev && logdev->logfile)
    {
#ifndef PC
        coines_set_pin_config(COINES_APP30_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
#endif
        logbin_add_data(sid, tns, event_size, event_payload, logdev);
#ifndef PC
        coines_set_pin_config(COINES_APP30_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
#endif
    }
}

static void stream_hex_data(uint8_t sid, uint32_t ts, uint32_t tns, uint8_t event_size, uint8_t *event_payload)
{
    /* Print sensor ID */
    HEX("%02x%08x%08x", sid, ts, tns);

    for (uint16_t i = 0; i < event_size; i++)
    {
        /* Output raw data in hex */
        PRINT_H("%02x", event_payload[i]);
    }

    PRINT_D("\r\n");
}

struct parse_sensor_details *parse_get_sensor_details(uint8_t id, struct parse_ref *ref)
{
    uint8_t i = 0;

    for (i = 0; i < BHY2_MAX_SIMUL_SENSORS; i++)
    {
        if (ref->sensor[i].id == id)
        {
            return &ref->sensor[i];
        }
    }

    return NULL;
}

struct parse_sensor_details *parse_add_sensor_details(uint8_t id, struct parse_ref *ref)
{
    uint8_t i = 0;

    struct parse_sensor_details *sensor_details;

    sensor_details = parse_get_sensor_details(id, ref);
    if (sensor_details)
    {

        /* Slot for the sensor ID is already used */
        return sensor_details;
    }
    else
    {
        /* Find a new slot */
        for (i = 0; i < BHY2_MAX_SIMUL_SENSORS; i++)
        {
            if (ref->sensor[i].id == 0)
            {
                INFO("Using slot %u for SID %u\r\n", i, id);
                ref->sensor[i].id = id;

                return &ref->sensor[i];
            }
        }
    }

    return NULL;
}

/* BHY2_SYS_ID_META_EVENT, BHY2_SYS_ID_META_EVENT_WU */
void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    uint32_t s, ns;
    uint64_t tns;
    char *event_text;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

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

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    switch (meta_event_type)
    {
        case BHY2_META_EVENT_FLUSH_COMPLETE:
            DATA("%s; T: %lu.%09lu; Flush complete for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
            DATA("%s; T: %lu.%09lu; Sample rate changed for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHY2_META_EVENT_POWER_MODE_CHANGED:
            DATA("%s; T: %lu.%09lu; Power mode changed for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHY2_META_EVENT_ALGORITHM_EVENTS:
            DATA("%s; T: %lu.%09lu; Algorithm event\r\n", event_text, s, ns);
            break;
        case BHY2_META_EVENT_SENSOR_STATUS:
            DATA("%s; T: %lu.%09lu; Accuracy for sensor id %u changed to %u\r\n", event_text, s, ns, byte1, byte2);
            sensor_details = parse_get_sensor_details(byte1, parse_table);
            if (parse_table && sensor_details)
            {
                sensor_details->accuracy = byte2;
            }
            else
            {
                INFO("Parse slot not defined for %u\r\n", byte1);
            }

            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
            DATA("%s; T: %lu.%09lu; BSX event (do steps main)\r\n", event_text, s, ns);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
            DATA("%s; T: %lu.%09lu; BSX event (do steps calib)\r\n", event_text, s, ns);
            break;
        case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            DATA("%s; T: %lu.%09lu; BSX event (get output signal)\r\n", event_text, s, ns);
            break;
        case BHY2_META_EVENT_SENSOR_ERROR:
            DATA("%s; T: %lu.%09lu; Sensor id %u reported error 0x%02X\r\n", event_text, s, ns, byte1, byte2);
            break;
        case BHY2_META_EVENT_FIFO_OVERFLOW:
            DATA("%s; T: %lu.%09lu; FIFO overflow\r\n", event_text, s, ns);
            break;
        case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
            DATA("%s; T: %lu.%09lu; Dynamic range changed for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHY2_META_EVENT_FIFO_WATERMARK:
            DATA("%s; T: %lu.%09lu; FIFO watermark reached\r\n", event_text, s, ns);
            break;
        case BHY2_META_EVENT_INITIALIZED:
            DATA("%s; T: %lu.%09lu; Firmware initialized. Firmware version %u\r\n", event_text, s, ns,
                 ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHY2_META_TRANSFER_CAUSE:
            DATA("%s; T: %lu.%09lu; Transfer cause for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHY2_META_EVENT_SENSOR_FRAMEWORK:
            DATA("%s; T: %lu.%09lu; Sensor framework event for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHY2_META_EVENT_RESET:
            DATA("%s; T: %lu.%09lu; Reset event. Cause : %u\r\n", event_text, s, ns, byte2);
            break;
        case BHY2_META_EVENT_SPACER:
            break;
        default:
            DATA("%s; T: %lu.%09lu; Unknown meta event with id: %u\r\n", event_text, s, ns, meta_event_type);
            break;
    }
}

void parse_3axis_s16(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhy2_data_xyz data;
    uint32_t s, ns;
    uint64_t tns;
    uint8_t parse_flag;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    float scaling_factor;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    scaling_factor = sensor_details->scaling_factor;
    parse_flag = sensor_details->parse_flag;

    bhy2_parse_xyz(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f; acc: %u\r\n",
             callback_info->sensor_id,
             s,
             ns,
             data.x * scaling_factor,
             data.y * scaling_factor,
             data.z * scaling_factor,
             sensor_details->accuracy);
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

void parse_euler(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhy2_data_orientation data;
    uint32_t s, ns;
    uint64_t tns;
    uint8_t parse_flag;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    float scaling_factor;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    scaling_factor = sensor_details->scaling_factor;
    parse_flag = sensor_details->parse_flag;

    bhy2_parse_orientation(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; h: %f, p: %f, r: %f; acc: %u\r\n",
             callback_info->sensor_id,
             s,
             ns,
             data.heading * scaling_factor,
             data.pitch * scaling_factor,
             data.roll * scaling_factor,
             sensor_details->accuracy);
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

void parse_quaternion(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhy2_data_quaternion data;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    bhy2_parse_quaternion(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f, w: %f; acc: %f\r\n",
             callback_info->sensor_id,
             s,
             ns,
             data.x / 16384.0f,
             data.y / 16384.0f,
             data.z / 16384.0f,
             data.w / 16384.0f,
             ((data.accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

void parse_s16_as_float(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    int16_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    float scaling_factor = 0.0f;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not define for %u\r\n", callback_info->sensor_id);

        return;
    }

    scaling_factor = sensor_details->scaling_factor;
    parse_flag = sensor_details->parse_flag;

    data = BHY2_LE2S16(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; %f\r\n", callback_info->sensor_id, s, ns, data * scaling_factor);
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

void parse_scalar_u32(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    data = BHY2_LE2U32(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; %lu\r\n", callback_info->sensor_id, s, ns, data);
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

void parse_scalar_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu;\r\n", callback_info->sensor_id, s, ns);
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

void parse_activity(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint16_t activity;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    activity = BHY2_LE2U16(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {

        DATA("SID: %u; T: %lu.%09lu; ", callback_info->sensor_id, s, ns);

        if (activity & BHY2_STILL_ACTIVITY_ENDED)
        {
            PRINT_D(" Still activity ended,");
        }

        if (activity & BHY2_WALKING_ACTIVITY_ENDED)
        {
            PRINT_D(" Walking activity ended,");
        }

        if (activity & BHY2_RUNNING_ACTIVITY_ENDED)
        {
            PRINT_D(" Running activity ended,");
        }

        if (activity & BHY2_ON_BICYCLE_ACTIVITY_ENDED)
        {
            PRINT_D(" On bicycle activity ended,");
        }

        if (activity & BHY2_IN_VEHICLE_ACTIVITY_ENDED)
        {
            PRINT_D(" In vehicle ended,");
        }

        if (activity & BHY2_TILTING_ACTIVITY_ENDED)
        {
            PRINT_D(" Tilting activity ended,");
        }

        if (activity & BHY2_STILL_ACTIVITY_STARTED)
        {
            PRINT_D(" Still activity started,");
        }

        if (activity & BHY2_WALKING_ACTIVITY_STARTED)
        {
            PRINT_D(" Walking activity started,");
        }

        if (activity & BHY2_RUNNING_ACTIVITY_STARTED)
        {
            PRINT_D(" Running activity started,");
        }

        if (activity & BHY2_ON_BICYCLE_ACTIVITY_STARTED)
        {
            PRINT_D(" On bicycle activity started,");
        }

        if (activity & BHY2_IN_VEHICLE_ACTIVITY_STARTED)
        {
            PRINT_D(" In vehicle activity started,");
        }

        if (activity & BHY2_TILTING_ACTIVITY_STARTED)
        {
            PRINT_D(" Tilting activity started,");
        }

        PRINT_D("\r\n");
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

void parse_u16_as_float(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint16_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    float scaling_factor = 0.0f;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    scaling_factor = sensor_details->scaling_factor;
    parse_flag = sensor_details->parse_flag;

    data = BHY2_LE2U16(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; %f\r\n", callback_info->sensor_id, s, ns, data * scaling_factor);
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

void parse_u24_as_float(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    float scaling_factor = 0.0f;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    scaling_factor = sensor_details->scaling_factor;
    parse_flag = sensor_details->parse_flag;

    data = BHY2_LE2U24(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; %f\r\n", callback_info->sensor_id, s, ns, (float)data * scaling_factor);
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

void parse_proximity(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    char *text;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;

    text = callback_info->data_ptr[0] ? "near" : "far";

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; %s\r\n", callback_info->sensor_id, s, ns, text);
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

void parse_scalar_u8(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint8_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    data = callback_info->data_ptr[0];
    parse_flag = sensor_details->parse_flag;

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; %u\r\n", callback_info->sensor_id, s, ns, data);
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

void parse_generic(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; D: ", callback_info->sensor_id, s, ns);
        for (uint8_t i = 0; i < (callback_info->data_size - 1); i++)
        {
            PRINT_D("%02X", callback_info->data_ptr[i]);
        }

        PRINT_D("\r\n");
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

void parse_device_ori(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    char *ori;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

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

    parse_flag = sensor_details->parse_flag;

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; %s\r\n", callback_info->sensor_id, s, ns, ori);
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

void parse_gps(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;
    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    callback_info->data_ptr[callback_info->data_size - 2] = '\0';

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("[GPS]; T: %lu.%09lu; data: %s\r\n", s, ns, callback_info->data_ptr);
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

void parse_debug_message(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    uint8_t msg_length = 0;
    uint8_t debug_msg[17] = { 0 }; /* Max payload size is 16 bytes, adds a trailing zero if the payload is full */

    if (!callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    msg_length = callback_info->data_ptr[0];

    memcpy(debug_msg, &callback_info->data_ptr[1], msg_length);
    debug_msg[msg_length] = '\0'; /* Terminate the string */

    DATA("[DEBUG MSG]; T: %lu.%09lu; %s\r\n", s, ns, debug_msg);
}
