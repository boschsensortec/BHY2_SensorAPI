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
 * @file    dinject.c
 * @brief   Source file for the data injection functions for the command line utility
 *
 */

#include <stdio.h>
#include "parse.h"
#include "bhy2_parse.h"
#include "verbose.h"
#include "coines.h"

#include <sys/stat.h>
#include "dinject.h"

/**
* @brief Function to initialize Data Injection structure
*/
int8_t dinject_init(uint8_t id, char *input, struct data_inject *dinject, struct bhy2_dev *bhy2)
{
    struct bhy2_sensor_info bhy2_sensor_info_t;
    struct stat st;
    int8_t rslt = BHY2_OK;

    /*! Read the Sensor Info */
    rslt = bhy2_get_sensor_info(id, &bhy2_sensor_info_t, bhy2);
    if (rslt != BHY2_OK)
    {
        return rslt;
    }

    /*! Open the Field Log*/
    dinject->in_log_ptr = fopen(input, "r");
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

    /*! Initialize the Data Injection Structure*/
    dinject->file_size = dinject->file_size - LINE_LENGTH(TIMESTAMP_LENGTH); /*subtracting 19, size of first line time
                                                                              * stamp-> */
    dinject->sensor_data_size = bhy2_sensor_info_t.event_size + 2;
    dinject->sensor_block_size = MAX_SAMPLES_PER_BLOCK * dinject->sensor_data_size;
    dinject->total_line = dinject->file_size / (LINE_LENGTH(dinject->sensor_data_size)); /*52 is the size of each line
                                                                                          * in the log file */

    return BHY2_OK;
}

/**
* @brief Function to get the Timestamp and read a data sample
*/
int8_t dinject_log_init(struct data_inject *dinject, struct bhy2_dev *bhy2)
{
    uint8_t log[dinject->sensor_block_size];
    uint8_t ts[TIMESTAMP_LENGTH];
    int8_t rslt;

    /*! Read large Timestamp */
    for (int i = 0; i < TIMESTAMP_LENGTH; i++)
    {
        fscanf(dinject->in_log_ptr, "%x", &ts[i]);
    }

    /*! Read Delta timestamp, accelerometer and gyroscope data*/
    for (int i = 0; i < dinject->sensor_data_size; i++)
    {
        fscanf(dinject->in_log_ptr, "%x", &log[i]);
    }

    /*! Decrement Line count */
    dinject->total_line = dinject->total_line - 1;

    /*! Inject the Large Timestamp*/
    rslt = bhy2_inject_data(ts, TIMESTAMP_LENGTH, bhy2);
    if (rslt != BHY2_OK)
    {
        ERROR("Timestamp Injection failed\r\n");

        return DINJECT_FAILED;
    }
    else
    {
        PRINT("Timestamp Injection successful\r\n");
    }

    /*! Inject the Data Sample*/
    rslt = bhy2_inject_data(log, dinject->sensor_data_size, bhy2);
    if (rslt != BHY2_OK)
    {
        ERROR("Data Sample Injection failed\r\n");

        return DINJECT_FAILED;
    }
    else
    {
        PRINT("Data Sample Injection successful\r\n");
    }

    return BHY2_OK;
}

/**
* @brief Function to inject the data
*/
int8_t dinject_inject_data(uint32_t *actual_len, struct data_inject *dinject, struct bhy2_dev *bhy2)
{
    uint8_t log[dinject->sensor_block_size];
    int8_t rslt;
    uint32_t len = *actual_len;

    /*! Read Bulk sensor data (7 packets max) */
    for (int i = 0; i < len; i++)
    {
        fscanf(dinject->in_log_ptr, "%x", &log[i]);
    }

    /*! Inject the Data*/
    rslt = bhy2_inject_data(log, len, bhy2);
    if (rslt != BHY2_OK)
    {
        ERROR("Data Injection failed\r\n");

        return DINJECT_FAILED;
    }

    /*! Update the Line count and check Data Injection progress*/
    dinject->total_line = dinject->total_line - (dinject->sensor_block_size / dinject->sensor_data_size);

    if (dinject->total_line <= 0)
    {
        PRINT("Data Injection completed successfully !!!!\r\n");

        return DINJECT_SUCCESSFUL;
    }
    else if (dinject->total_line < (dinject->sensor_block_size / dinject->sensor_data_size))
    {
        len = dinject->total_line * dinject->sensor_data_size;
    }
    else
    {
        len = dinject->sensor_block_size;
    }

    return DINJECT_IN_PROGRESS;
}

/**
* @brief Function to uninitialize the Data Injection structure
*/
int8_t dinject_deinit(struct data_inject *dinject, struct bhy2_dev *bhy2)
{
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
