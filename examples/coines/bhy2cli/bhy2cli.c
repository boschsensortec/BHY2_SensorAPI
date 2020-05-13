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
 * @file    bhy2cli.c
 * @date    24 Mar 2020
 * @brief   Command line utility for the BHI260/BHA260
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>

#include "bhy2.h"
#include "bhy2_parse.h"
#include "parse.h"
#include "common.h"

static uint8_t verbose = 2;

#define PRINT(format, ...)   printf(format,##__VA_ARGS__)
#define INFO(format, ...)    if (verbose >= 2) printf("[Info]"format,##__VA_ARGS__)
#define PRINT_I(format, ...) if (verbose >= 2) printf(format,##__VA_ARGS__)
#define WARNING(format, ...) if (verbose >= 1) printf("[Warning]"format,##__VA_ARGS__)

/*#define PRINT_W(format, ...) if (verbose >= 1) printf(format,##__VA_ARGS__) */
#define ERROR(format, ...)   printf("[Error]"format,##__VA_ARGS__)

/*#define PRINT_E(format, ...) printf(format,##__VA_ARGS__) */

#define BHY2_ASSERT(x)       assert_rslt = x; if (assert_rslt) bhy2_check(__LINE__, __FUNCTION__, assert_rslt)

#define BHY2CLI_MAX_STRING_LENGTH UINT32_C(32)
#define BHY2_RD_WR_LEN 44

struct bhy2_dev bhy2;
volatile uint8_t need_to_stop = 0, sensor_parsing_en = 0;
bool sensors_present[256];
uint8_t fifo_buffer[2048];
int8_t assert_rslt;

struct parse_ref parse_table;

/* Contains all parameters of the of custom virtual sensors
 * required for parsing */
typedef struct custom_driver_information
{
    char sensor_name[BHY2CLI_MAX_STRING_LENGTH];
    uint16_t sensor_payload;
    uint8_t is_registered;
    uint16_t sensor_id;
    char output_formats[BHY2CLI_MAX_STRING_LENGTH];
} custom_driver_information_t;

/* Global table that contains the payloads of present custom virtual sensors, derived by a parameter read. */
custom_driver_information_t custom_driver_information[(BHY2_SENSOR_ID_CUSTOM_END - BHY2_SENSOR_ID_CUSTOM_START) + 1];

void bhy2_check(unsigned int line, const char *func, int8_t val)
{
    ERROR("BHI260 API failed at line %u. The function %s returned error code %d. %s\r\n",
          line,
          func,
          val,
          get_api_error(val));
    exit(1);
}

void bhy2_get_present_custom_sensors(void)
{
    int8_t rslt;
    uint8_t present_buff[32];

    rslt = bhy2_get_virt_sensor_list(present_buff, &bhy2);
    BHY2_ASSERT(rslt);

    for (uint16_t i = 0; i < sizeof(present_buff); i++)
    {
        for (uint8_t j = 0; j < 8; j++)
        {
            sensors_present[i * 8 + j] = ((present_buff[i] >> j) & 0x01);
        }
    }
}

void print_boot_status(uint8_t boot_status)
{
    INFO("Boot Status : 0x%02x: ", boot_status);
    if (boot_status & BHY2_BST_FLASH_DETECTED)
    {
        PRINT_I("Flash detected. ");
    }
    if (boot_status & BHY2_BST_FLASH_VERIFY_DONE)
    {
        PRINT_I("Flash verify done. ");
    }
    if (boot_status & BHY2_BST_FLASH_VERIFY_ERROR)
    {
        PRINT_I("Flash verification failed. ");
    }
    if (boot_status & BHY2_BST_NO_FLASH)
    {
        PRINT_I("No flash installed. ");
    }
    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        PRINT_I("Host interface ready. ");
    }
    if (boot_status & BHY2_BST_HOST_FW_VERIFY_DONE)
    {
        PRINT_I("Firmware verification done. ");
    }
    if (boot_status & BHY2_BST_HOST_FW_VERIFY_ERROR)
    {
        PRINT_I("Firmware verification error. ");
    }
    if (boot_status & BHY2_BST_HOST_FW_IDLE)
    {
        PRINT_I("Firmware halted. ");
    }
    PRINT_I("\r\n");
}

void bhy2_parse_custom_sensor_default(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    uint8_t this_sensor_id;
    uint8_t this_sensor_payload;

    this_sensor_id = callback_info->sensor_id;
    this_sensor_payload =
        (uint8_t)custom_driver_information[this_sensor_id - BHY2_SENSOR_ID_CUSTOM_START].sensor_payload;

    if (this_sensor_payload > callback_info->data_size)
    {
        ERROR("Mismatch in payload size\r\n");
        exit(1);
    }

    /* Print sensor ID */
    INFO("%u; ", this_sensor_id);

    for (uint16_t i = 0; i < this_sensor_payload - 1; i++)
    {
        /* Output raw data in hex */
        PRINT_I("%x ", callback_info->data_ptr[i]);
    }
    PRINT_I("\r\n");
}

void bhy2_parse_custom_sensor(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    uint8_t idx;
    char *strtok_ptr;
    char *parameter_delimiter = ":";
    char tmp_output_formats[BHY2CLI_MAX_STRING_LENGTH];
    uint8_t rel_sensor_id; /* Relative sensor ID  */
    uint32_t s, ns;
    union
    {
        uint32_t data_u32;
        float data_float;
    }
    u32_to_float;

    uint8_t tmp_u8 = 0;
    uint16_t tmp_u16 = 0;
    uint32_t tmp_u32 = 0;
    int8_t tmp_s8 = 0;
    int16_t tmp_s16 = 0;
    int32_t tmp_s32 = 0;
    uint8_t tmp_data_c = 0;

    /* Get sensor_id to access correct parsing information from global linked list  */
    rel_sensor_id = callback_info->sensor_id - BHY2_SENSOR_ID_CUSTOM_START;

    /* Fetch output_formats string from linked list */
    strcpy(tmp_output_formats, custom_driver_information[rel_sensor_id].output_formats);
    strtok_ptr = strtok(tmp_output_formats, parameter_delimiter);

    uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */
    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

    if ((custom_driver_information[rel_sensor_id].sensor_payload + 1) != callback_info->data_size)
    {
        ERROR("Mismatch in payload size\r\n");
        exit(1);
    }

    /* Print sensor id and timestamp */
    INFO("%u; %u.%09u; ", callback_info->sensor_id, s, ns);

    /* Parse output_formats and output data depending on the individual format of an output */
    idx = 0;
    while (strtok_ptr != NULL)
    {

        if (strcmp(strtok_ptr, "u8") == 0)
        {
            tmp_u8 = callback_info->data_ptr[idx];
            idx += 1;

            PRINT_I("%u ", tmp_u8);
        }
        else if (strcmp(strtok_ptr, "u16") == 0)
        {
            tmp_u16 = BHY2_LE2U16(&callback_info->data_ptr[idx]);
            idx += 2;

            PRINT_I("%u ", tmp_u16);
        }
        else if (strcmp(strtok_ptr, "u32") == 0)
        {
            tmp_u32 = BHY2_LE2U32(&callback_info->data_ptr[idx]);
            idx += 4;

            PRINT_I("%u ", tmp_u32);
        }
        else if (strcmp(strtok_ptr, "s8") == 0)
        {
            tmp_s8 = (int8_t)callback_info->data_ptr[idx];
            idx += 1;

            PRINT_I("%d ", tmp_s8);
        }
        else if (strcmp(strtok_ptr, "s16") == 0)
        {
            tmp_s16 = BHY2_LE2S16(&callback_info->data_ptr[idx]);
            idx += 2;

            PRINT_I("%d ", tmp_s16);
        }
        else if (strcmp(strtok_ptr, "s32") == 0)
        {
            tmp_s32 = BHY2_LE2S32(&callback_info->data_ptr[idx]);
            idx += 4;

            PRINT_I("%d ", tmp_s32);
        }
        else if (strcmp(strtok_ptr, "c") == 0)
        {
            tmp_data_c = callback_info->data_ptr[idx];
            idx += 1;

            PRINT_I("%c ", tmp_data_c);
        }
        else if (strcmp(strtok_ptr, "f") == 0)
        {
            /* Float values have to be read as unsigned and then interpreted as float */
            u32_to_float.data_u32 = BHY2_LE2U32(&callback_info->data_ptr[idx]);
            idx += 4;

            /* The binary data has to be interpreted as a float */
            PRINT_I("%6.4f ", u32_to_float.data_float);
        }

        strtok_ptr = strtok(NULL, parameter_delimiter);
    }

    PRINT_I("\r\n");

    if (idx != custom_driver_information[rel_sensor_id].sensor_payload)
    {
        ERROR("Provided Output format sizes don't add up to total sensor payload!\r\n");
        exit(1);
    }
}

void cli_cmd_do_add_driver(const char *payload)
{
    char *start;
    char *end;
    char str_sensor_id[BHY2CLI_MAX_STRING_LENGTH];
    char str_sensor_payload[BHY2CLI_MAX_STRING_LENGTH];
    char output_formats[BHY2CLI_MAX_STRING_LENGTH];
    char sensor_name[BHY2CLI_MAX_STRING_LENGTH];
    uint8_t sensor_id;
    uint8_t sensor_payload;
    uint8_t len_of_output_formats;
    struct bhy2_sensor_info sensor_info;

    start = (char*)payload;
    end = strchr(start, ':');

    if (end == NULL)
    {
        ERROR("Add Sensor format error\r\n");
        exit(1);
    }

    /* Parse sensor ID */

    /* Check length of string */
    if (((uint32_t)(end - start)) > BHY2CLI_MAX_STRING_LENGTH)
    {
        ERROR("Too many characters for sensor ID!\r\n");
        exit(1);
    }
    strncpy(str_sensor_id, start, (size_t)(end - start));
    str_sensor_id[end - start] = '\0';

    /* Convert string to int */
    sensor_id = (uint8_t)strtol(str_sensor_id, NULL, 10);
    INFO("Sensor ID: %u \r\n", sensor_id);

    /* Parse sensor name */
    start = end + 1;
    end = strchr(start, ':');
    if (end == NULL)
    {
        ERROR("Add Sensor name error\r\n");
        exit(1);
    }

    /* Check length of string */
    if (((uint32_t)(end - start)) > BHY2CLI_MAX_STRING_LENGTH)
    {
        ERROR("Too many characters for sensor name. Only %u characters allowed\r\n", BHY2CLI_MAX_STRING_LENGTH);
        exit(1);
    }
    strncpy(sensor_name, start, (size_t)(end - start));
    sensor_name[end - start] = '\0';
    INFO("Sensor Name: %s \r\n", sensor_name);

    /* Parse sensor payload */
    start = end + 1;
    end = strchr(start, ':');

    if (end == NULL)
    {
        ERROR("Add Sensor payload error\r\n");
        exit(1);
    }

    strncpy(str_sensor_payload, start, (size_t)(end - start));
    str_sensor_payload[end - start] = '\0';
    sensor_payload = (uint8_t)strtol(str_sensor_payload, NULL, 10);
    INFO("Sensor Payload: %u \r\n", sensor_payload);

    /* Parse output formats string, final parsing of each output is done in the parsing callback function */
    start = end + 1;
    len_of_output_formats = (uint8_t)strlen(start);
    end = start + len_of_output_formats;
    memcpy(output_formats, start, (size_t)((size_t)(end - start)));
    output_formats[end - start] = '\0';

    /* Get the sensor information */
    BHY2_ASSERT(bhy2_get_sensor_info(sensor_id, &sensor_info, &bhy2));

    /* Check if supplied payload matches the event size. Note event size includes sensor id in the payload */
    if (sensor_info.event_size != (sensor_payload + 1))
    {
        ERROR("Provided total payload size of sensor ID %u doesn't match the actual payload size!\r\n", sensor_id);
        exit(1);
    }

    /* Store parsed data into the custom driver information table */
    custom_driver_information[sensor_id - BHY2_SENSOR_ID_CUSTOM_START].sensor_id = sensor_id;
    custom_driver_information[sensor_id - BHY2_SENSOR_ID_CUSTOM_START].sensor_payload = sensor_payload;
    strncpy(custom_driver_information[sensor_id - BHY2_SENSOR_ID_CUSTOM_START].output_formats,
            output_formats,
            BHY2CLI_MAX_STRING_LENGTH);
    custom_driver_information[sensor_id - BHY2_SENSOR_ID_CUSTOM_START].is_registered = 1;
    strncpy(custom_driver_information[sensor_id - BHY2_SENSOR_ID_CUSTOM_START].sensor_name,
            sensor_name,
            BHY2CLI_MAX_STRING_LENGTH);

    /* Register the custom sensor callback function*/
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(sensor_id, bhy2_parse_custom_sensor, &parse_table, &bhy2));

    INFO("Adding custom driver payload successful\r\n");
}

void install_callbacks(void)
{
    for (uint8_t i = 1; i < BHY2_SENSOR_ID_MAX; i++)
    {
        parse_table.sensor[i].scaling_factor = 1.0f;
        BHY2_ASSERT(bhy2_register_fifo_parse_callback(i, parse_generic, &parse_table, &bhy2));
    }

    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, &parse_table, &bhy2));

    parse_table.sensor[BHY2_SENSOR_ID_ACC].scaling_factor = 1.0f / 4096.0f;
    parse_table.sensor[BHY2_SENSOR_ID_ACC_WU].scaling_factor = 1.0f / 4096.0f;
    parse_table.sensor[BHY2_SENSOR_ID_ACC_PASS].scaling_factor = 1.0f / 4096.0f;
    parse_table.sensor[BHY2_SENSOR_ID_ACC_RAW].scaling_factor = 1.0f / 4096.0f;
    parse_table.sensor[BHY2_SENSOR_ID_ACC_RAW_WU].scaling_factor = 1.0f / 4096.0f;
    parse_table.sensor[BHY2_SENSOR_ID_ACC_BIAS].scaling_factor = 1.0f / 4096.0f;
    parse_table.sensor[BHY2_SENSOR_ID_GRA].scaling_factor = 1.0f / 4096.0f;
    parse_table.sensor[BHY2_SENSOR_ID_GRA_WU].scaling_factor = 1.0f / 4096.0f;
    parse_table.sensor[BHY2_SENSOR_ID_LACC].scaling_factor = 1.0f / 4096.0f;
    parse_table.sensor[BHY2_SENSOR_ID_LACC_WU].scaling_factor = 1.0f / 4096.0f;

    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ACC, parse_3axis_s16, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ACC_WU, parse_3axis_s16, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ACC_PASS, parse_3axis_s16, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ACC_RAW, parse_3axis_s16, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ACC_RAW_WU, parse_3axis_s16, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ACC_BIAS, parse_3axis_s16, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GRA, parse_3axis_s16, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GRA_WU, parse_3axis_s16, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_LACC, parse_3axis_s16, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_LACC_WU, parse_3axis_s16, &parse_table, &bhy2));

    parse_table.sensor[BHY2_SENSOR_ID_MAG].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_MAG_WU].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_MAG_PASS].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_MAG_RAW].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_MAG_RAW_WU].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_MAG_BIAS].scaling_factor = 1.0f;

    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_MAG, parse_3axis_s16, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_MAG_WU, parse_3axis_s16, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_MAG_PASS, parse_3axis_s16, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_MAG_RAW, parse_3axis_s16, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_MAG_RAW_WU, parse_3axis_s16, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_MAG_BIAS, parse_3axis_s16, &parse_table, &bhy2));

    parse_table.sensor[BHY2_SENSOR_ID_GYRO].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_GYRO_WU].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_GYRO_PASS].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_GYRO_RAW].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_GYRO_RAW_WU].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_GYRO_BIAS].scaling_factor = 1.0f;

    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GYRO, parse_3axis_s16, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GYRO_WU, parse_3axis_s16, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GYRO_PASS, parse_3axis_s16, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GYRO_RAW, parse_3axis_s16, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GYRO_RAW_WU, parse_3axis_s16, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GYRO_BIAS, parse_3axis_s16, &parse_table, &bhy2));

    parse_table.sensor[BHY2_SENSOR_ID_ORI].scaling_factor = 360.0f / 32768.0f;
    parse_table.sensor[BHY2_SENSOR_ID_ORI_WU].scaling_factor = 360.0f / 32768.0f;
    parse_table.sensor[BHY2_SENSOR_ID_RV].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_RV_WU].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_GAMERV].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_GAMERV_WU].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_GEORV].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_GEORV_WU].scaling_factor = 1.0f;

    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ORI, parse_euler, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ORI_WU, parse_euler, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_RV, parse_quaternion, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_RV_WU, parse_quaternion, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GAMERV, parse_quaternion, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GAMERV_WU, parse_quaternion, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GEORV, parse_quaternion, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GEORV_WU, parse_quaternion, &parse_table, &bhy2));

    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_SIG, parse_scalar_event, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_SIG_HW, parse_scalar_event, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_SIG_HW_WU, parse_scalar_event, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_STD, parse_scalar_event, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_STD_WU, parse_scalar_event, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_STD_HW, parse_scalar_event, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_STD_HW_WU, parse_scalar_event, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_STC, parse_scalar_u32, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_STC_WU, parse_scalar_u32, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_STC_HW, parse_scalar_u32, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_STC_HW_WU, parse_scalar_u32, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_TILT_DETECTOR, parse_scalar_event, &parse_table,
                                                  &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_WAKE_GESTURE, parse_scalar_event, &parse_table,
                                                  &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GLANCE_GESTURE, parse_scalar_event, &parse_table,
                                                  &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_PICKUP_GESTURE, parse_scalar_event, &parse_table,
                                                  &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_AR, parse_activity, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GPS, parse_gps, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ANY_MOTION, parse_scalar_event, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ANY_MOTION_WU, parse_scalar_event, &parse_table,
                                                  &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_EXCAMERA, parse_scalar_u8, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_DEVICE_ORI, parse_device_ori, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_DEVICE_ORI_WU, parse_device_ori, &parse_table, &bhy2));

    parse_table.sensor[BHY2_SENSOR_ID_TEMP].scaling_factor = 1.0f / 100.0f;
    parse_table.sensor[BHY2_SENSOR_ID_TEMP_WU].scaling_factor = 1.0f / 100.0f;
    parse_table.sensor[BHY2_SENSOR_ID_LIGHT].scaling_factor = 10000.0f / 216.0f;
    parse_table.sensor[BHY2_SENSOR_ID_LIGHT_WU].scaling_factor = 10000.0f / 216.0f;
    parse_table.sensor[BHY2_SENSOR_ID_BARO].scaling_factor = 100.0f / 128.0f;
    parse_table.sensor[BHY2_SENSOR_ID_BARO_WU].scaling_factor = 100.0f / 128.0f;
    parse_table.sensor[BHY2_SENSOR_ID_PROX].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_PROX_WU].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_HUM].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_HUM_WU].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_GAS].scaling_factor = 1.0f;
    parse_table.sensor[BHY2_SENSOR_ID_GAS_WU].scaling_factor = 1.0f;

    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_TEMP, parse_s16_as_float, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_TEMP_WU, parse_s16_as_float, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_LIGHT, parse_s16_as_float, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_LIGHT_WU, parse_s16_as_float, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_BARO, parse_u24_as_float, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_BARO_WU, parse_u24_as_float, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_PROX, parse_scalar_u8, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_PROX_WU, parse_scalar_u8, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_HUM, parse_scalar_u8, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_HUM_WU, parse_scalar_u8, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GAS, parse_scalar_u32, &parse_table, &bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GAS_WU, parse_scalar_u32, &parse_table, &bhy2));

    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SYS_ID_DEBUG_MSG, parse_debug_message, &parse_table, &bhy2));
}

void print_usage(void)
{
    PRINT_I("Usage:\r\n");
    PRINT_I("bhy2cli [<options>]\r\n");
    PRINT_I("Options:\r\n");
    PRINT_I("  -h\t= Print this usage message\r\n");
    PRINT_I("  -i\t= Show device information: Device ID,\r\n");
    PRINT_I("    \t  ROM version, RAM version, Power state,\r\n");
    PRINT_I("    \t  list of available sensors,\r\n");
    PRINT_I("    \t  content of Boot Status register,\r\n");
    PRINT_I("    \t  content of Error value register\r\n");
    PRINT_I("  -v <verbose level>\r\n");
    PRINT_I("    \t= Set the verbose level. 0 Error, 1 Warning, 2 Infos\r\n");
    PRINT_I("  -b <firmware path>\r\n");
    PRINT_I("    \t= Reset, upload specified firmware to RAM and boot from RAM\r\n");
    PRINT_I("    \t  [equivalent to using -n -u <firmware> -g r successively]\r\n");
    PRINT_I("  -d <firmware path>\r\n");
    PRINT_I("    \t= Reset, upload specified firmware to Flash and boot from Flash\r\n");
    PRINT_I("    \t  [equivalent to using -n -f <firmware> -g f successively]\r\n");
    PRINT_I("  -n\t= Reset sensor hub\r\n");
    PRINT_I("  -a <sensor id>:<sensor name>:<total output payload in bytes>:\r\n");
    PRINT_I("     <output_format_0>:<output_format_1>\r\n");
    PRINT_I("    \t= Register the expected payload of a new custom virtual sensor\r\n");
    PRINT_I("    \t -Valid output_formats: u8: Unsigned 8 Bit, u16: Unsigned 16 Bit, u32:\r\n");
    PRINT_I("    \t  Unsigned 32 Bit, s8: Signed 8 Bit, 16: Signed 16 Bit, s32: Signed 32 Bit,\r\n");
    PRINT_I("    \t  f: Float, c: Char \r\n");
    PRINT_I("    \t -e.g.: -a 160:\"Lean Orientation\":2:c:c \r\n");
    PRINT_I("    \t -Note that the corresponding virtual sensor has to be enabled in the same function\r\n");
    PRINT_I("    \t  call (trailing -c option), since the registration of the sensor is temporary. \r\n");
    PRINT_I("  -r <adr>[:<len>]\r\n");
    PRINT_I("    \t= Read from register address <adr> for length <len> bytes\r\n");
    PRINT_I("    \t -If input <len> is not provided, the default read length is 1 byte\r\n");
    PRINT_I("    \t -When reading registers with auto-increment, the provided register as well as\r\n");
    PRINT_I("    \t  the following registers will be read\r\n");
    PRINT_I("    \t -e.g -r 0x08:3 will read the data of registers 0x08, 0x09 and 0x0a\r\n");
    PRINT_I("    \t  max. 53 bytes can be read at once\r\n");
    PRINT_I("  -w <adr>=<val1>[,<val2>]...\r\n");
    PRINT_I("    \t= Write to register address <adr> with comma separated values <val>\r\n");
    PRINT_I("    \t -If more values provided <val>, the additional\r\n");
    PRINT_I("    \t  values will be written to the following addresses\r\n");
    PRINT_I("    \t -When writing to registers with auto-increment, the provided register as well as\r\n");
    PRINT_I("    \t  the following registers will be written\r\n");
    PRINT_I("    \t -e.g -w 0x08=0x02,0x03,0x04 will write the provided data to registers 0x08, 0x09\r\n");
    PRINT_I("    \t  and 0x0a. Max. 46 bytes can be written at once\r\n");
    PRINT_I("  -s <param id>\r\n");
    PRINT_I("    \t= Display read_param response of parameter <param id>\r\n");
    PRINT_I("  -t <param id>=<val1>[,<val2>]...\r\n");
    PRINT_I("    \t= Write data to parameter <param id> with the bytes to be written, <val1>[,<val2>]... \r\n");
    PRINT_I("    \t -e.g. 0x103=5,6 will write 0x05 to the first byte and 0x06 to the second byte\r\n");
    PRINT_I("    \t  of the parameter \"Fifo Control\"\r\n");
    PRINT_I("  -u <firmware path>\r\n");
    PRINT_I("    \t= Upload firmware to RAM\r\n");
    PRINT_I("  -f <firmware path>\r\n");
    PRINT_I("    \t= Upload firmware to external-flash\r\n");
    PRINT_I("  -g <medium>\r\n");
    PRINT_I("    \t= Boot from the specified <medium>: \"f\" for FLASH, \"r\" for RAM\r\n");
    PRINT_I("  -e\t= Erase external-flash\r\n");
    PRINT_I("  -c <sensor id>:<frequency>[:<latency>]\r\n");
    PRINT_I("    \t= Activate sensor <sensor id> at specified sample rate <frequency>,\r\n");
    PRINT_I("    \t -latency <latency>, duration time <time>, sample counts <count>\r\n");
    PRINT_I("    \t -At least <frequency> is a must input parameter\r\n");
    PRINT_I("    \t -<latency> is optional\r\n");
    PRINT_I("    \t -One or more sensors can be active by passing multiple -c options\r\n");
    PRINT_I("    \t -id: sensor id\r\n");
    PRINT_I("    \t -frequency(Hz): sensor ODR\r\n");
    PRINT_I("    \t -latency(ms): sensor data outputs with a latency\r\n");
}

void cli_cmd_do_show_information(void)
{
    uint16_t kernel_version = 0, user_version = 0;
    uint16_t rom_version = 0;
    uint8_t product_id = 0;
    uint8_t host_status = 0;
    uint8_t val = 0;
    uint8_t sensor_error;

    /* Get product_id */
    BHY2_ASSERT(bhy2_get_product_id(&product_id, &bhy2));

    /* Get Kernel version */
    BHY2_ASSERT(bhy2_get_kernel_version(&kernel_version, &bhy2));

    /* Get User version */
    BHY2_ASSERT(bhy2_get_user_version(&user_version, &bhy2));

    /* Get ROM version */
    BHY2_ASSERT(bhy2_get_rom_version(&rom_version, &bhy2));

    BHY2_ASSERT(bhy2_get_host_status(&host_status, &bhy2));

    INFO("Product ID     : %02x\r\n", product_id);
    INFO("Kernel version : %04u\r\n", kernel_version);
    INFO("User version   : %04u\r\n", user_version);
    INFO("ROM version    : %04u\r\n", rom_version);
    INFO("Power state    : %s\r\n", (host_status & BHY2_HST_POWER_STATE) ? "sleeping" : "active");
    INFO("Host interface : %s\r\n", (host_status & BHY2_HST_HOST_PROTOCOL) ? "SPI" : "I2C");

    /* Read boot status */
    BHY2_ASSERT(bhy2_get_boot_status(&val, &bhy2));
    print_boot_status(val);

    /* Read error value */
    BHY2_ASSERT(bhy2_get_error_value(&sensor_error, &bhy2));
    if (sensor_error)
    {
        ERROR("%s\r\n", get_sensor_error_text(sensor_error));
    }

    if (kernel_version)
    {
        /* Get present virtual sensor */
        bhy2_get_present_custom_sensors();

        INFO("Virtual sensor list.\r\n");
        INFO("Sensor ID\t| Sensor Name\r\n");
        for (uint8_t i = 0; i < BHY2_SENSOR_ID_MAX; i++)
        {
            if (sensors_present[i])
            {
                if (i < BHY2_SENSOR_ID_CUSTOM_START)
                {
                    INFO(" %5u \t| %s\r\n", i, get_sensor_name(i));
                }
                else
                {
                    INFO(" %5u \t| %s\r\n", i, custom_driver_information[i - BHY2_SENSOR_ID_CUSTOM_START].sensor_name);
                }
            }
        }
    }
}

void cli_cmd_do_reset_hub(void)
{
    uint8_t data = 0, data_exp;

    BHY2_ASSERT(bhy2_soft_reset(&bhy2));

	BHY2_ASSERT(bhy2_get_host_interrupt_ctrl(&data, &bhy2));
	data &= ~BHY2_ICTL_DISABLE_STATUS_FIFO; /* Enable status interrupts */
	data &= ~BHY2_ICTL_DISABLE_DEBUG; /* Enable debug interrupts */
	data_exp = data;
    BHY2_ASSERT(bhy2_set_host_interrupt_ctrl(data, &bhy2));
    BHY2_ASSERT(bhy2_get_host_interrupt_ctrl(&data, &bhy2));
    if (data != data_exp)
    {
        WARNING("Expected Host Interrupt Control (0x07) to have value 0x%x but instead read 0x%x\r\n", data_exp, data);
    }

    /* Config status channel */
    BHY2_ASSERT(bhy2_set_host_intf_ctrl(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL, &bhy2));
    BHY2_ASSERT(bhy2_get_host_intf_ctrl(&data, &bhy2));
    if (!(data & BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL))
    {
        WARNING("Expected Host Interface Control (0x06) to have bit 0x%x to be set\r\n",
                BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL);
    }

    INFO("Reset successful\r\n");
}

void cli_cmd_do_upload_to_ram(const char *filepath)
{
    FILE *fp;
    uint8_t *firmware_image;
    uint32_t len;
    size_t ret;
    int8_t rslt = BHY2_OK;
    uint8_t boot_status;
    uint32_t start_time_ms;

    BHY2_ASSERT(bhy2_get_boot_status(&boot_status, &bhy2));
    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        if ((boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) || (boot_status & BHY2_BST_FLASH_VERIFY_DONE))
        {
            ERROR("Seems like a firmware is running. Please reset the BHI260/BHA260 before uploading firmware\r\n");
            exit(1);
        }

        fp = fopen(filepath, "rb");
        if (!fp)
        {
            ERROR("Cannot open file: %s\r\n", filepath);
            exit(1);
        }
        fseek(fp, 0, SEEK_END);
        len = (uint32_t)ftell(fp);
        rewind(fp);
        firmware_image = (uint8_t*)malloc(len);
        ret = fread(firmware_image, sizeof(uint8_t), len, fp);
        fclose(fp);
        if (ret != (size_t)len)
        {
            free(firmware_image);
            ERROR("Reading file failed\r\n");
            exit(1);
        }

        INFO("Uploading %u bytes of firmware to RAM\r\n", len);
        start_time_ms = coines_get_millis();
        uint32_t incr = BHY2_RD_WR_LEN;
        if ((incr % 4) != 0)
        {
            incr = ((incr >> 2) + 1) << 2;
        }
        for (uint32_t i = 0; (i < len) && (rslt == BHY2_OK); i += incr)
        {
            if (incr > (len - i)) /* Last payload */
            {
                incr = len - i;
                if ((incr % 4) != 0) /* Round off to higher 4 bytes */
                {
                    incr = ((incr >> 2) + 1) << 2;
                }
            }
            rslt = bhy2_upload_firmware_to_ram_partly(&firmware_image[i], len, i, incr, &bhy2);
            INFO("Completed %.2f%%\r", (float)(i + incr) / (float)len * 100.0f);
        }
        PRINT_I("\n");
        INFO("Firmware upload took %.2f seconds\r\n", (float)(coines_get_millis() - start_time_ms) / 1000.0f);
        free(firmware_image);
        if (rslt != BHY2_OK)
        {
            ERROR("Firmware upload failed. Returned with error code: %d. %s\r\n", rslt, get_api_error(rslt));
            exit(1);
        }
    }
    else
    {
        ERROR("Host interface is not ready\r\n");
        exit(1);
    }

    INFO("Uploading firmware to RAM successful\r\n");
}

void cli_cmd_do_boot_from_ram()
{
    int8_t rslt;
    uint16_t kernel_version;
    uint8_t boot_status, error_val;
    uint16_t tries = 100;

    INFO("Waiting for firmware verification to complete\r\n");
    do
    {
        bhy2.hif.delay_us(10000, NULL);
        BHY2_ASSERT(bhy2_get_boot_status(&boot_status, &bhy2));
        if (boot_status & BHY2_BST_HOST_FW_VERIFY_DONE)
        {
            break;
        }
    } while (tries--);

    print_boot_status(boot_status);
    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        if (boot_status & BHY2_BST_HOST_FW_VERIFY_DONE)
        {
            BHY2_ASSERT(bhy2_get_kernel_version(&kernel_version, &bhy2));
            if (!kernel_version)
            {

                rslt = bhy2_boot_from_ram(&bhy2);
                if (rslt != BHY2_OK)
                {
                    ERROR("Booting from RAM failed. API error code %d.\r\n%s\r\n", rslt, get_api_error(rslt));
                    BHY2_ASSERT(bhy2_get_error_value(&error_val, &bhy2));
                    if (error_val)
                    {
                        ERROR("Sensor reports error 0x%X.\r\n%s", error_val, get_sensor_error_text(error_val));
                    }
                    exit(1);
                }
            }

            BHY2_ASSERT(bhy2_get_kernel_version(&kernel_version, &bhy2));
            if (kernel_version)
            {
                BHY2_ASSERT(bhy2_update_virtual_sensor_list(&bhy2));

                for (uint16_t i = 0; i < 10; i++)
                {
                    /* Process meta events over a period of 100ms*/
                    BHY2_ASSERT(bhy2_get_and_process_fifo(fifo_buffer, sizeof(fifo_buffer), &bhy2));
                    bhy2.hif.delay_us(10000, NULL);
                }
            }
            else
            {
                ERROR("Reading Kernel version failed, booting from RAM failed\r\n");
                exit(1);
            }
        }
        else
        {
            ERROR("Upload firmware to RAM before boot\r\n");
            exit(1);
        }

    }
    else
    {
        ERROR("Host interface is not ready\r\n");
        exit(1);
    }

    INFO("Booting from RAM successful\r\n");
}

void cli_cmd_do_erase_flash(uint32_t end_addr)
{
    int8_t rslt;
    uint8_t boot_status;

    BHY2_ASSERT(bhy2_get_boot_status(&boot_status, &bhy2));
    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        if ((boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) || (boot_status & BHY2_BST_FLASH_VERIFY_DONE))
        {
            ERROR("Seems like a firmware is running. Reset the BHI260/BHA260 before erasing external flash\r\n");
            exit(1);
        }
    }
    INFO("Erasing flash. May take a while\r\n");
    rslt = bhy2_erase_flash(BHY2_FLASH_SECTOR_START_ADDR, BHY2_FLASH_SECTOR_START_ADDR + end_addr, &bhy2);
    if (rslt != BHY2_OK)
    {
        ERROR("Erasing flash failed, status: %02d\r\n", rslt);
        exit(1);
    }

    INFO("Erasing flash successful\r\n");
}

void cli_cmd_do_upload_to_flash(const char *filepath)
{
    FILE *fp;
    uint8_t *firmware_image;
    uint32_t len;
    size_t ret;
    int8_t rslt = BHY2_OK;
    uint8_t boot_status;
    uint32_t start_time_ms;

    BHY2_ASSERT(bhy2_get_boot_status(&boot_status, &bhy2));
    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        if ((boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) || (boot_status & BHY2_BST_FLASH_VERIFY_DONE))
        {
            ERROR("Reset the BHI260/BHA260 before uploading firmware to external flash\r\n");
            exit(1);
        }

        fp = fopen(filepath, "rb");
        if (!fp)
        {
            ERROR("Cannot open file: %s\r\n", filepath);
            exit(1);
        }
        fseek(fp, 0, SEEK_END);
        len = (uint32_t)ftell(fp);
        rewind(fp);
        firmware_image = (uint8_t*)malloc(len);
        ret = fread(firmware_image, sizeof(uint8_t), len, fp);
        fclose(fp);
        if (ret != (size_t)len)
        {
            free(firmware_image);
            ERROR("Reading file failed\r\n");
            exit(1);
        }

        INFO("Erasing first %u bytes of flash\r\n", len);
        rslt = bhy2_erase_flash(BHY2_FLASH_SECTOR_START_ADDR, BHY2_FLASH_SECTOR_START_ADDR + len, &bhy2);
        if (rslt != BHY2_OK)
        {
            ERROR("Erasing flash failed with error code %d. %s\r\n", rslt, get_api_error(rslt));
        }

        INFO("Uploading %u bytes of firmware to flash\r\n", len);
        start_time_ms = coines_get_millis();
        uint32_t incr = BHY2_RD_WR_LEN;
        if ((incr % 4) != 0) /* Round of to closest 4bytes */
        {
            incr = ((incr >> 2) + 1) << 2;
        }
        for (uint32_t i = 0; (i < len) && (rslt == BHY2_OK); i += incr)
        {
            if (incr > (len - i)) /* Last payload */
            {
                incr = len - i;
                if ((incr % 4) != 0) /* Round of to closest 4bytes */
                {
                    incr = ((incr >> 2) + 1) << 2;
                }
            }
            rslt = bhy2_upload_firmware_to_flash_partly(&firmware_image[i], i, incr, &bhy2);
            INFO("Completed %.2f%%\r", (float)(i + incr) / (float)len * 100.0f);
        }
        PRINT_I("\n");
        INFO("Firmware upload took %.2f seconds\r\n", (float)(coines_get_millis() - start_time_ms) / 1000.0f);
        free(firmware_image);
        if (rslt != BHY2_OK)
        {
            ERROR("%s. Firmware upload failed\r\n", get_api_error(rslt));
            exit(1);
        }
    }
    else
    {
        ERROR("Host interface is not ready\r\n");
        exit(1);
    }

    INFO("Uploading firmware to flash successful\r\n");
}

void cli_cmd_do_boot_from_flash()
{
    int8_t rslt;
    uint16_t kernel_version;
    uint8_t boot_status;
    uint8_t error_val = 0;
    uint16_t tries = 300; /* Wait for up to little over 3s */

    INFO("Waiting for firmware verification to complete\r\n");
    do
    {
        bhy2.hif.delay_us(10000, NULL);
        BHY2_ASSERT(bhy2_get_boot_status(&boot_status, &bhy2));
        if (boot_status & BHY2_BST_FLASH_VERIFY_DONE)
        {
            break;
        }
    } while (tries--);

    BHY2_ASSERT(bhy2_get_boot_status(&boot_status, &bhy2));
    print_boot_status(boot_status);
    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        if (boot_status & BHY2_BST_FLASH_DETECTED)
        {
            BHY2_ASSERT(bhy2_get_kernel_version(&kernel_version, &bhy2));
            if (!kernel_version)
            {
                /* If no firmware is running, boot from Flash */
                INFO("Booting from flash\r\n");
                rslt = bhy2_boot_from_flash(&bhy2);
                if (rslt != BHY2_OK)
                {
                    ERROR("%s. Booting from flash failed.\r\n", get_api_error(rslt));
                    BHY2_ASSERT(bhy2_get_regs(BHY2_REG_ERROR_VALUE, &error_val, 1, &bhy2));
                    if (error_val)
                    {
                        ERROR("%s\r\n", get_sensor_error_text(error_val));
                    }
                    exit(1);
                }

                BHY2_ASSERT(bhy2_get_boot_status(&boot_status, &bhy2));
                print_boot_status(boot_status);

                if (!(boot_status & BHY2_BST_HOST_INTERFACE_READY))
                {
                    /* hub is not ready, need reset hub */
                    INFO("Host interface is not ready, triggering a reset\r\n");

                    BHY2_ASSERT(bhy2_soft_reset(&bhy2));
                }
            }

            BHY2_ASSERT(bhy2_get_kernel_version(&kernel_version, &bhy2));
            if (kernel_version)
            {
                BHY2_ASSERT(bhy2_update_virtual_sensor_list(&bhy2));
                for (uint16_t i = 0; i < 10; i++) /* Process meta events */
                {
                    BHY2_ASSERT(bhy2_get_and_process_fifo(fifo_buffer, sizeof(fifo_buffer), &bhy2));
                    bhy2.hif.delay_us(10000, NULL);
                }
            }
            else
            {
                ERROR("Reading Kernel version failed, booting from flash failed\r\n");
                exit(1);
            }
        }
        else
        {
            ERROR("Can't detect external flash\r\n");
            exit(1);
        }
    }
    else
    {
        ERROR("Host interface is not ready\r\n");
        exit(1);
    }

    INFO("Booting from flash successful\r\n");
}

void cli_cmd_do_activate(const char *sensor_parameters)
{
    char sen_id_str[8], sample_rate_str[8], sen_latency_str[8];
    uint8_t sen_id;
    uint32_t sen_latency = 0;
    float sample_rate;
    char *start, *end;

    /* Parse Sensor ID */
    start = (char*)sensor_parameters;
    end = strchr(start, ':');
    if (end == NULL)
    {
        ERROR("Sensor ID / Sample rate format error\r\n");
        exit(1);
    }

    strncpy(sen_id_str, start, (size_t)(end - start));
    sen_id_str[end - start] = '\0';
    sen_id = (uint8_t)atoi(sen_id_str);

    /* Parse sample rate */
    start = end + 1;
    end = strchr(start, ':');

    if (end == NULL)
    {
        end = start + strlen(start);
    }

    strncpy(sample_rate_str, start, (size_t)(end - start));
    sample_rate_str[end - start] = '\0';
    sample_rate = (float)atof(sample_rate_str);

    if (sample_rate < 0)
    {
        sample_rate = 0.0f;
    }

    /*  Parse Latency */
    if (strlen(end))
    {
        start = end + 1;
        end = strchr(start, ':');

        if (end == NULL)
        {
            end = start + strlen(start);
        }

        strncpy(sen_latency_str, start, (size_t)(end - start));
        sen_latency_str[end - start] = '\0';
        sen_latency = (uint32_t)atoi(sen_latency_str);
    }

    BHY2_ASSERT(bhy2_update_virtual_sensor_list(&bhy2));

    /* If the payload of this sensor is not yet registered and within the custom virtual sensor id range, register the
     * default parsing function */
    if (sensors_present[sen_id])
    {
        if (custom_driver_information[sen_id - BHY2_SENSOR_ID_CUSTOM_START].is_registered != 1)
        {
            /* If sensor id is within the customer reserved range */
            if ((sen_id >= BHY2_SENSOR_ID_CUSTOM_START) && (sen_id <= BHY2_SENSOR_ID_CUSTOM_END))
            {
                custom_driver_information[sen_id -
                                          BHY2_SENSOR_ID_CUSTOM_START].sensor_payload = bhy2.table[sen_id].event_size;

                BHY2_ASSERT(bhy2_register_fifo_parse_callback(sen_id, bhy2_parse_custom_sensor_default, NULL, &bhy2));
                INFO("No output interpretation has been provided for this sensor. ");
                PRINT_I("FIFO data will be printed as hex values. ");
                PRINT_I("For registering the payload interpretation, use the -a option\r\n");
            }

        }
    }
    else
    {
        ERROR("The requested sensor is not present in the loaded firmware!\r\n");
        exit(1);
    }

    INFO("Sensor ID: %u, sample rate: %f Hz, latency: %u ms\r\n", sen_id, sample_rate, sen_latency);

    /* Flush sensor data from the FIFO */
    BHY2_ASSERT(bhy2_flush_fifo(sen_id, &bhy2));

    /* Enable sensor and set sample rate */
    BHY2_ASSERT(bhy2_set_virt_sensor_cfg(sen_id, sample_rate, sen_latency, &bhy2));

    /* Sensor data will be parsed and printed after processing all arguments */
}

void cli_cmd_do_read_from_register_address(const char *payload)
{
    char *start;
    char *end;
    char str_reg[8] = { 0 };
    uint8_t reg;
    uint8_t val[1024];
    uint16_t len;
    uint16_t i = 0;
    uint16_t j = 0;

    start = (char*)payload;
    end = strchr(start, ':');
    if (end == NULL)
    {
        end = start + strlen(start);
    }

    /* Parse register address */
    strncpy(str_reg, start, (size_t)(end - start));
    str_reg[end - start] = '\0';
    reg = (uint8_t)strtol(str_reg, NULL, 0);

    /* Parse read length, rest of the string */
    start = end + 1;
    len = (uint16_t)strtol(start, NULL, 0);

    /* Default read length is 1 */
    if (len < 1)
    {
        len = 1;
    }

    /* Execution of bus read function */
    BHY2_ASSERT(bhy2_get_regs(reg, val, len, &bhy2));

    /* Print register data to console */

    /* Registers after the status channel are auto increment,
     * reading more than 1 byte will lead to reading
     * the subsequent register addresses */
    if (reg <= BHY2_REG_CHAN_STATUS)
    {
        INFO("Reading from register address 0x%02x:\r\n", reg);
        INFO("Byte hex       dec | Data\r\n");
        INFO("-------------------------------------------\r\n");
        for (i = 0; i < len; i++)
        {
            if (j == 0)
            {
                INFO(" ");
                PRINT_I("0x%06x %8d |", i, i);
            }
            PRINT_I(" %02x", val[i]);
            ++j;
            if (j >= 8)
            {
                PRINT_I("\r\n");
                j = 0;
            }
        }
        if ((len % 8) == 0)
        {
            PRINT_I("\r\n");
        }
    }
    else
    {
        INFO("Register address: Data\r\n");
        INFO("----------------------\r\n");
        for (i = 0; i < len; i++)
        {
            INFO("0x%02x            : %02x \r\n", reg + i, val[i]);
        }
    }
    INFO("Read complete\r\n");
}

void cli_cmd_do_write_to_register_address(const char *payload)
{
    char *start;
    char *end;
    char str_reg[8] = { 0 };
    uint8_t reg;
    uint8_t val[1024] = { 0 };
    char *strtok_ptr;
    char *byte_delimiter = ",";
    uint16_t len = 0;

    /* Parse register address */
    start = (char*)payload;
    end = strchr(start, '=');
    if (end == NULL)
    {
        ERROR("Write address format error\r\n");
        exit(1);
    }
    strncpy(str_reg, start, (size_t)(end - start));
    str_reg[end - start] = '\0';
    reg = (uint8_t)strtol(str_reg, NULL, 0);

    /* Parse values to be written */
    start = end + 1;
    strtok_ptr = strtok(start, byte_delimiter);

    while (strtok_ptr != NULL)
    {
        val[len] = (uint8_t)strtol(strtok_ptr, NULL, 0);
        strtok_ptr = strtok(NULL, byte_delimiter);
        ++len;
    }

    /* Execution of bus write function */
    BHY2_ASSERT(bhy2_set_regs(reg, val, len, &bhy2));

    INFO("Writing address successful\r\n");
}

void cli_cmd_do_read_param(const char *payload)
{
    char str_param_id[8] = { 0 };
    uint8_t tmp_buf[1024] = { 0 };
    uint16_t param_id;
    uint32_t ret_len = 0;
    uint16_t i;
    uint16_t j = 0;

    memcpy(str_param_id, payload, strlen(payload));
    str_param_id[strlen(payload)] = '\0';
    param_id = (uint16_t)strtol(str_param_id, NULL, 0);

    BHY2_ASSERT(bhy2_get_parameter(param_id, tmp_buf, sizeof(tmp_buf), &ret_len, &bhy2));
    INFO("Byte hex      dec | Data\r\n");
    INFO("-------------------------------------------\r\n");
    for (i = 0; i < ret_len; i++)
    {
        if (j == 0)
        {
            INFO("0x%06x %8d |", i, i);
        }
        PRINT_I("%02x ", tmp_buf[i]);
        j++;
        if (j >= 8)
        {
            PRINT_I("\r\n");
            j = 0;
        }
    }
    if ((ret_len % 8) != 0)
    {
        PRINT_I("\r\n");
    }

    INFO("Reading parameter 0x%04X successful\r\n", param_id);
}

void cli_cmd_do_write_param(const char *payload)
{
    char *start, *end;
    char str_param_id[8] = { 0 };
    char str_data[8] = { 0 };
    uint8_t data_buf[1024] = { 0 };
    uint16_t param_id;
    uint8_t val;
    uint16_t buf_size = 0;
    uint8_t break_out = 0;

    start = (char*)payload;
    end = strchr(start, '=');
    if (end == NULL)
    {
        ERROR("Write parameter I/O format error\r\n");
        exit(1);
    }
    strncpy(str_param_id, start, (size_t)(end - start));
    str_param_id[end - start] = '\0';
    param_id = (uint16_t)strtol(str_param_id, NULL, 0);

    /* Parse write data */
    do
    {
        start = end + 1;
        end = strchr(start, ',');
        if (end == NULL)
        {
            end = start + strlen(start);
            break_out++;
        }
        memcpy(str_data, start, (size_t)(end - start));
        str_data[end - start] = '\0';
        val = (uint8_t)strtol(str_data, NULL, 0);
        data_buf[buf_size] = val;
        buf_size++;
    } while (!break_out);

    /* Make sure write buffer size is always multiples of 4 */
    if (buf_size % 4)
    {
        buf_size = (uint16_t)((buf_size / 4 + 1) * 4);
    }

    BHY2_ASSERT(bhy2_set_parameter(param_id, data_buf, buf_size, &bhy2));
    INFO("Writing parameter successful\r\n");
}

void sigint_handler(int sig_num)
{
    (void)sig_num;
    (void)signal(SIGINT, NULL);
    need_to_stop = 1;
    INFO("Exiting\r\n");
    if (!sensor_parsing_en)
    {
        INFO("Force exit\r\n");
        exit(0);
    }
}

int main(int argc, const char *const argv[])
{
    /* Index command line arguments */
    uint8_t arg_i;

    /* Used to skip parsing the first command if help command was detected */
    uint8_t for_start = 1;
    uint8_t sen_en_check = 0;

    /* Used to keep track of sensors activated via -c command */
    uint8_t number_of_activated_sensors = 0;

    parse_table.verbose = &verbose;
    uint8_t expected_data;

    /* Disable printf buffering for stdout */
    setbuf(stdout, NULL);

    for (uint8_t i = BHY2_SENSOR_ID_CUSTOM_START; i <= BHY2_SENSOR_ID_CUSTOM_END; i++)
    {
        custom_driver_information[i - BHY2_SENSOR_ID_CUSTOM_START].is_registered = 0;
        strcpy(custom_driver_information[i - BHY2_SENSOR_ID_CUSTOM_START].sensor_name, "Undefined custom sensor");
    }

    /* Print Copyright build date */
    PRINT("Copyright (c) 2020 Bosch Sensortec GmbH\r\n");
    PRINT("Build date: " __DATE__ "\r\n");

    if (argc <= 1)
    {
        ERROR("No argument supplied!\nPlease refer to the following usage message:\r\n\r\n");
        print_usage();
        exit(1);
    }

    /* Parse -h before starting board communication -> help can be displayed without a board attached */
    if (argv[1][1] == 'h')
    {
        print_usage();

        /* If help is recognized, don't parse first option of command anymore */
        for_start++;
    }

    (void)signal(SIGINT, sigint_handler);

    setup_interfaces(false, BHY2_SPI_INTERFACE);

    BHY2_ASSERT(bhy2_init(BHY2_SPI_INTERFACE, bhy2_spi_read, bhy2_spi_write, bhy2_delay_us, BHY2_RD_WR_LEN, NULL, &bhy2));

    /* Check hub state */
    BHY2_ASSERT(bhy2_get_product_id(&expected_data, &bhy2));

    uint8_t product_id;
    BHY2_ASSERT(bhy2_get_product_id(&product_id, &bhy2));
    if (product_id == BHY2_PRODUCT_ID)
    {
        uint16_t kernel_version;
        BHY2_ASSERT(bhy2_get_kernel_version(&kernel_version, &bhy2));
        INFO("Device found\r\n");
        if (kernel_version)
        {
            INFO("Firmware running\r\n");

            bhy2_get_present_custom_sensors();
        }
        else
        {
            INFO("No firmware running\r\n");
        }
    }
    else
    {
        ERROR("Device not found, Check connections and power. Product ID read 0x%x\r\n", product_id);
        exit(1);
    }

    /* Config status channel */
    BHY2_ASSERT(bhy2_set_host_intf_ctrl(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL, &bhy2));
    BHY2_ASSERT(bhy2_get_host_intf_ctrl(&expected_data, &bhy2));
    if (!(expected_data & BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL))
    {
        WARNING("Expected Host Interface Control (0x06) to have bit 0x%x to be set\r\n",
                BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL);
    }

    /* Install virtual sensor callbacks */
    install_callbacks();

    /* Go through all command line arguments and take action according to supplied commands */
    /*lint -e{850} i is modified in the body of the for loop */
    for (arg_i = for_start; arg_i < argc; arg_i++)
    {
        if (argv[arg_i][0] != '-' && argv[arg_i][0] != '/')
        {
            ERROR("Invalid command: %s\r\n", argv[arg_i]);
            exit(1);
        }

        switch (argv[arg_i][1])
        {
            case 'h':
                print_usage();
                break;
            case 'v':
                arg_i++;
                verbose = (uint8_t)(argv[arg_i][0] - '0');
                if (verbose > 2)
                {
                    verbose = 2;
                }
                INFO("Setting verbose to  %u\r\n", verbose);
                break;
            case 'b':
                arg_i++;
                if (arg_i >= argc)
                {
                    ERROR("Firmware path expected\r\n");
                    exit(1);
                }
                INFO("Executing -b %s\r\n", argv[arg_i]);
                cli_cmd_do_reset_hub();
                cli_cmd_do_upload_to_ram(argv[arg_i]);
                cli_cmd_do_boot_from_ram();
                break;
            case 'd':
                arg_i++;
                if (arg_i >= argc)
                {
                    ERROR("Firmware path expected\r\n");
                    exit(1);
                }
                INFO("Executing -d %s\r\n", argv[arg_i]);
                cli_cmd_do_reset_hub();
                cli_cmd_do_upload_to_flash(argv[arg_i]);
                cli_cmd_do_boot_from_flash();
                break;
            case 'n':
            	INFO("Executing -n\r\n");
                cli_cmd_do_reset_hub();
                break;
            case 'a':
                arg_i++;
                if (arg_i >= argc)
                {
                    ERROR("Sensor ID and Output Format details expected\r\n");
                    exit(1);
                }
                INFO("Executing -a %s\r\n", argv[arg_i]);
                cli_cmd_do_add_driver(argv[arg_i]);
                break;
            case 'g':
                arg_i++;
                INFO("Executing -g %s\r\n", argv[arg_i]);
                if ((argv[arg_i][0]) == 'r')
                {
                    cli_cmd_do_boot_from_ram();
                }
                else if ((argv[arg_i][0]) == 'f')
                {
                    cli_cmd_do_boot_from_flash();
                }
                else
                {
                    ERROR("Invalid boot medium: %s\r\n", argv[arg_i]);
                    exit(1);
                }
                break;
            case 'c':
                arg_i++;
                if (arg_i >= argc)
                {
                    ERROR("Sensor ID and ODR expected\r\n");
                    exit(1);
                }
                INFO("Executing -c %s\r\n", argv[arg_i]);
                cli_cmd_do_activate(argv[arg_i]);
                sensor_parsing_en = 1;
                number_of_activated_sensors++;
                break;
            case 'e':
            	INFO("Executing -e\r\n");
                cli_cmd_do_erase_flash(BHY2_FLASH_SIZE_4MB);
                break;
            case 'r':
                arg_i++;
                if (arg_i >= argc)
                {
                    ERROR("Register and length expected\r\n");
                    exit(1);
                }
                INFO("Executing -r %s\r\n", argv[arg_i]);
                cli_cmd_do_read_from_register_address(argv[arg_i]);
                break;
            case 'u':
                arg_i++;
                INFO("Executing -u %s\r\n", argv[arg_i]);
                cli_cmd_do_upload_to_ram(argv[arg_i]);
                break;
            case 'f':
                arg_i++;
                if (arg_i >= argc)
                {
                    ERROR("Firmware path expected\r\n");
                    exit(1);
                }
                INFO("Executing -f %s\r\n", argv[arg_i]);
                cli_cmd_do_upload_to_flash(argv[arg_i]);
                break;
            case 'w':
                arg_i++;
                if (arg_i >= argc)
                {
                    ERROR("Register and data expected\r\n");
                    exit(1);
                }
                INFO("Executing -w %s\r\n", argv[arg_i]);
                cli_cmd_do_write_to_register_address(argv[arg_i]);
                break;
            case 'i':
            	INFO("Executing -i\r\n");
                cli_cmd_do_show_information();
                break;
            case 's':
                arg_i++;
                if (arg_i >= argc)
                {
                    ERROR("Parameter ID expected\r\n");
                    exit(1);
                }
                INFO("Executing -s %s\r\n", argv[arg_i]);
                cli_cmd_do_read_param(argv[arg_i]);
                break;
            case 't':
                arg_i++;
                if (arg_i >= argc)
                {
                    ERROR("Parameter ID and data expected\r\n");
                    exit(1);
                }
                INFO("Executing -t %s\r\n", argv[arg_i]);
                cli_cmd_do_write_param(argv[arg_i]);
                break;
            default:
                ERROR("Unrecognized command %s!\r\n", argv[arg_i]);
                break;
        }
    }

    /* If -c option was used, sensor data will be streamed after parsing and executing all commands */
    if (sensor_parsing_en == 1)
    {
        /* Process data until signal to stop is received */
        sen_en_check = 0;
        for (uint16_t i = 0; i < BHY2_SENSOR_ID_MAX; i++)
        {
            if (sensors_present[i])
            {
                sen_en_check = 1;
            }
        }
        if (sen_en_check == 0)
        {
            need_to_stop = 1;
        }

        while (!need_to_stop)
        {
            if (get_interrupt_status())
            {
                BHY2_ASSERT(bhy2_get_and_process_fifo(fifo_buffer, sizeof(fifo_buffer), &bhy2));
            }
            bhy2.hif.delay_us(1000, NULL);
        }

        for (uint8_t i = 0; i < BHY2_SENSOR_ID_MAX; i++)
        {
            if (sensors_present[i])
            {
                /* Deactivate sensor */
                BHY2_ASSERT(bhy2_set_virt_sensor_cfg(i, 0.0f, 0, &bhy2));
            }
        }

        /* Parse remaining FIFO data */
        for (uint16_t i = 0; i < number_of_activated_sensors * 10; i++)
        {
            BHY2_ASSERT(bhy2_get_and_process_fifo(fifo_buffer, sizeof(fifo_buffer), &bhy2));
            bhy2.hif.delay_us(2000, NULL);
        }
    }

    close_interfaces();

    exit(0);
}
