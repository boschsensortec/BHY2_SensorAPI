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
 * @file    swim_data_injection.c
 * @brief   SWIM Data injection - Integration validation
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "bhy2_parse.h"
#include "common.h"

#include "bhy2_hif.h"
#include <sys/stat.h>

/*! Enable this macro for bhy2 flash build */
//#define BHY2_FLASH_BUILD

#define SENSOR_BLOCK_DATA_SIZE 119
#define TIMESTAMP_LENGTH       UINT8_C(6)
#define SENSOR_DATA_SIZE       UINT8_C(17)
#define SYNCHRONOUS_MODE       UINT8_C(0)
#define WORK_BUFFER_SIZE       UINT16_C(2048)


/*! Sensor ID for SWIM sensor */
#define CUSTOM_SENSOR_ID      BHY2_SENSOR_ID_SWIM

/*! Start/Stop Swim sensor streaming */
#define SWIM_STOP_STREAMING   UINT8_C(1)
#define SWIM_START_STREAMING  UINT8_C(0)

/*!
 * @brief Output of the swim data is parsed for printing
 * @param[in] callback_info fifo data available here
 * @param[in] callback_ref
 *
 * @return  void
 *
 */
static void parse_swim(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

/*!
 * @brief Generated meta data events are printed
 * @param[in] callback_info fifo data available here
 * @param[in] callback_ref
 *
 * @return  void
 *
 */
static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

/*!
 * @brief Printing the error codes that generated from the API
 * @param[in] callback_info fifo data available here
 * @param[in] callback_ref
 *
 * @return  void
 *
 */
static void print_api_error(int8_t rslt, struct bhy2_dev *dev);

/*! File pointer to download firmware to BHI260AP */
static int8_t upload_firmware(const char *filepath, struct bhy2_dev *dev);

/*! File pointer to store the log */
FILE *fp;

/*! Device structure */
struct bhy2_dev bhy2;

/*! Structure to ge the output from the algorithm for logging */
struct bhy2_swim_algo_output data_out;
/*! Parameter for time in second and nano seconds */
uint32_t seconds, nano_seconds;

/*! SWIM configuration parameter structure which is
 * in sync with the parameter page of swim sensor 0x0B00 */
bhy2_swim_config_param_t bhy2_swim_configuration;

/*! Swim sensor can be disabled by pressing T1 button
 * which uses this global flag to disable the sensor */
uint8_t swim_data_en = SWIM_START_STREAMING;

enum bhy2_intf intf;

int main(int argc, char **argv)
{
    uint8_t product_id = 0;
    uint16_t version = 0;
    int8_t rslt;
    uint8_t work_buffer[WORK_BUFFER_SIZE];
    uint8_t work_buf[8];
    uint32_t actual_len = 0;
    uint16_t i = 0;
    uint8_t val = 0x0;
    uint16_t len = 1, code = 0;
    uint8_t hif_ctrl, boot_status;
    float sample_rate = 100.0; /*! Read out data measured at 100Hz */
    uint32_t report_latency_ms = 0; /*! Report immediately */
    struct stat st;
    int32_t total_line = 0;
    uint8_t log[SENSOR_BLOCK_DATA_SIZE];
    uint8_t inject_lts[TIMESTAMP_LENGTH];
    struct bhy2_sensor_info bhy2_sensor_info_t;

    if (argc != 6)
    {
        /* Firmware need to be added as args*/
        printf ("Usage: swim_data_injection.exe <firmware> <Injection_logname> <pool_length> <hand_selection> <output_file.csv> \r\n");
        printf ("       = <firmware> <Path-of-file> / <DataInject_nobsx.fw / DataInject_nobsx-flash.fw>\r\n");
        printf ("       = <Injection_logname>  <Path-of-file> / <field_trial_log_hex_converted.txt>\r\n");
        printf ("       = <Pool_length>  <25/50> meter\r\n");
        printf ("       = <Hand_selection>  <l/r>\r\n");
        printf ("       = <Output_file.csv> redirecting output from stdout to a csv file\r\n");
#ifdef BHY2_FLASH_BUILD
        printf ("Eg:    swim_data_injection.exe ../../../firmware/bhi260ap_swim/DataInject_nobsx-flash.fw SwimLog_20100120_130430.txt 25 l output_log.csv\r\n");
#else
        printf ("Eg:    swim_data_injection.exe ../../../firmware/bhi260ap_swim/DataInject_nobsx.fw SwimLog_20100120_130430.txt 25 l output_log.csv\r\n");
#endif
        exit(0);
    }

    const char *log_name = argv[2];
    /*! Wait here till serial port is opened  (or) BLE is connected (or) T2 button is pressed
     *  Selecting the SPI interface for sensor communication */
#ifdef BHY2_USE_I2C
    intf = BHY2_I2C_INTERFACE;
#else
    intf = BHY2_SPI_INTERFACE;
#endif

    setup_interfaces(true, intf); /* Perform a power on reset */

#ifdef BHY2_USE_I2C
    rslt = bhy2_init(BHY2_I2C_INTERFACE, bhy2_i2c_read, bhy2_i2c_write, bhy2_delay_us, BHY2_RD_WR_LEN, NULL, &bhy2);
#else
    rslt = bhy2_init(BHY2_SPI_INTERFACE, bhy2_spi_read, bhy2_spi_write, bhy2_delay_us, BHY2_RD_WR_LEN, NULL, &bhy2);
#endif
    print_api_error(rslt, &bhy2);

    rslt = bhy2_soft_reset(&bhy2);
    print_api_error(rslt, &bhy2);

    rslt = bhy2_get_product_id(&product_id, &bhy2);
    print_api_error(rslt, &bhy2);

    /*! File open for data logging */
    fp = fopen(argv[5], "w");

    /*! Check for a valid product ID */
    if (product_id != BHY2_PRODUCT_ID)
    {
        printf("Product ID read %X. Expected %X\r\n", product_id, BHY2_PRODUCT_ID);
    }
    else
    {
        printf("BHI260/BHA260 found. Product ID read %X\r\n", product_id);
    }

    /*! Configure the host interface */
    hif_ctrl = BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL;
    rslt = bhy2_set_host_intf_ctrl(hif_ctrl, &bhy2);
    print_api_error(rslt, &bhy2);

    /*! Check if the sensor is ready to load firmware */
    rslt = bhy2_get_boot_status(&boot_status, &bhy2);
    print_api_error(rslt, &bhy2);

    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        uint8_t sensor_error;
        int8_t temp_rslt;

        /*! If loading firmware to flash, erase the relevant section */
#ifdef BHY2_FLASH_BUILD
        if (boot_status & BHY2_BST_FLASH_DETECTED)
        {
        	struct stat st;
        	stat((const char*) argv[1], &st);
            uint32_t start_addr = BHY2_FLASH_SECTOR_START_ADDR;
            uint32_t end_addr = start_addr + st.st_size;
            printf("Flash detected. Erasing flash to upload firmware\r\n");

            rslt = bhy2_erase_flash(start_addr, end_addr, &bhy2);
            print_api_error(rslt, &bhy2);
        }
        else
        {
            printf("Flash not detected\r\n");

            return 0;
        }
#endif

        /*! Writing the firmware to the sensor memory */
        if (upload_firmware((const char*) argv[1], &bhy2))
        {
#ifdef BHY2_FLASH_BUILD
                printf("Booting from Flash.\r\n");
                rslt = bhy2_boot_from_flash(&bhy2);
#else
                printf("Booting from RAM.\r\n");
                rslt = bhy2_boot_from_ram(&bhy2);
#endif
        }

        temp_rslt = bhy2_get_error_value(&sensor_error, &bhy2);
        if (sensor_error)
        {
            printf("%s\r\n", get_sensor_error_text(sensor_error));
        }

        print_api_error(rslt, &bhy2);
        print_api_error(temp_rslt, &bhy2);

        rslt = bhy2_get_kernel_version(&version, &bhy2);
        print_api_error(rslt, &bhy2);
        if ((rslt == BHY2_OK) && (version != 0))
        {
            printf("Boot successful. Kernel version %u.\r\n", version);
        }

        /*! Registering the callback functions */
        rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, NULL, &bhy2);
        print_api_error(rslt, &bhy2);
        rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event, NULL, &bhy2);
        print_api_error(rslt, &bhy2);
        rslt = bhy2_register_fifo_parse_callback(CUSTOM_SENSOR_ID, parse_swim, NULL, &bhy2);
        print_api_error(rslt, &bhy2);
        rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy2);
        print_api_error(rslt, &bhy2);
    }
    else
    {
        printf("Host interface not ready. Exiting\r\n");

        close_interfaces(intf);

        return 0;
    }

    /*! Update the callback table to enable parsing of sensor data */
    rslt = bhy2_update_virtual_sensor_list(&bhy2);
    print_api_error(rslt, &bhy2);

    /*! Check for sensor Info */
    bhy2_get_sensor_info(CUSTOM_SENSOR_ID, &bhy2_sensor_info_t, &bhy2);
    printf("Sensor type=%d, driver id=%d, event size=%d bytes\r\n",
           bhy2_sensor_info_t.sensor_type,
           bhy2_sensor_info_t.driver_id,
           bhy2_sensor_info_t.event_size);
    fprintf(fp,
            "Sensor type=%d, driver id=%d, event size=%d bytes\r\n",
            bhy2_sensor_info_t.sensor_type,
            bhy2_sensor_info_t.driver_id,
            bhy2_sensor_info_t.event_size);

    /*! set pool length and on left/right hand configurations */
    /*! Possible Configuration field values>
     * SWIM_ENABLE_CONFIG
     * SWIM_DISABLE_CONFIG */
    bhy2_swim_configuration.update_swim_config = BHY2_SWIM_ENABLE_CONFIG;
    /*! Possible Hand selection values>
     * SWIM_DEVICE_ON_LEFT_HAND
     * SWIM_DEVICE_ON_RIGHT_HAND */
    bhy2_swim_configuration.dev_on_left_hand = (argv[4][0] == 'r') ? BHY2_SWIM_DEVICE_ON_RIGHT_HAND : BHY2_SWIM_DEVICE_ON_LEFT_HAND;

    bhy2_swim_configuration.pool_length_integral = atoi((const char*) argv[3]);

    bhy2_swim_configuration.pool_length_floating = 0; /*! Floating point support is yet to be added for variable pool
                                                       * length */

    /*! Writing the configuration to the parameter page */
    rslt = bhy2_swim_set_config(&bhy2_swim_configuration, &bhy2);
    print_api_error(rslt, &bhy2);

    /*! Set Step by step data injection mode */
    rslt = bhy2_set_data_injection_mode(BHY2_STEP_BY_STEP_INJECTION, &bhy2);
    print_api_error(rslt, &bhy2);

    /*! Setting the Sampling frequency and latency time */
    rslt = bhy2_set_virt_sensor_cfg(CUSTOM_SENSOR_ID, sample_rate, report_latency_ms, &bhy2);
    print_api_error(rslt, &bhy2);

    rslt = bhy2_hif_wait_status_ready(&bhy2.hif);
    print_api_error(rslt, &bhy2);

    rslt = bhy2_hif_get_status_fifo(&code, work_buf, sizeof(work_buf), &actual_len, &bhy2.hif);
    if (rslt != BHY2_OK)
    {
        printf("get_status fifo failed\r\n");
    }

    FILE *log_ptr = fopen(log_name, "r");
    if (!log_ptr)
    {
        printf("Cannot open file\r\n");

        return false;
    }
    stat(log_name, &st);
    uint32_t file_size = st.st_size;
    file_size = file_size - 19;//subtracting size of first line time stamp.
    total_line = file_size / 52;//52 is the size of each line in the log file
    actual_len = SENSOR_BLOCK_DATA_SIZE;

    /*! Read large Timestamp */
    for(i = 0; i< TIMESTAMP_LENGTH; i++){
        fscanf (log_ptr, "%x", &inject_lts[i]);
    }
    /*! Read Delta timestamp, accelerometer and gyroscope data*/
    for(i = 0; i< SENSOR_DATA_SIZE; i++){
        fscanf (log_ptr, "%x", &log[i]);
    }
    total_line = total_line - 1;

    rslt = bhy2_inject_data(inject_lts, TIMESTAMP_LENGTH, &bhy2);

    rslt = bhy2_inject_data(log, SENSOR_DATA_SIZE, &bhy2);

    /*! Data from the FIFO is read and the relevant callbacks if registered are called */
    while (rslt == BHY2_OK && swim_data_en != SWIM_STOP_STREAMING)
    {
        if (get_interrupt_status())
        {
            val = BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL;
            rslt = bhy2_hif_set_regs(BHY2_REG_HOST_INTERFACE_CTRL, &val, len, &bhy2.hif);
            print_api_error(rslt, &bhy2);
            /*! Data from the FIFO is read and the relevant callbacks if registered are called */
            rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy2);

            print_api_error(rslt, &bhy2);

            val = SYNCHRONOUS_MODE;
            /*! set sync mode */
            rslt = bhy2_hif_set_regs(BHY2_REG_HOST_INTERFACE_CTRL, &val, len, &bhy2.hif);
            print_api_error(rslt, &bhy2);
            /*! Read Bulk sensor data (7 packets max) */
            for(i = 0; i < actual_len; i++)
            {
                rslt = fscanf (log_ptr, "%x", &log[i]);
            }

            rslt = bhy2_inject_data(log, actual_len, &bhy2);
            if (rslt != BHY2_OK)
            {
                printf("bhy2_inject_data inside while failed\r\n");
                break;
            }
            total_line = total_line - (SENSOR_BLOCK_DATA_SIZE/SENSOR_DATA_SIZE);

            if (total_line <= 0)
            {
                printf ("injection completed successfully !!!!\r\n");
                break;
            }
            else if (total_line < (SENSOR_BLOCK_DATA_SIZE/SENSOR_DATA_SIZE))
            {
                actual_len = total_line * SENSOR_DATA_SIZE;
            }
            else
            {
                actual_len = SENSOR_BLOCK_DATA_SIZE;
            }
        }
    }

    printf("--- Swim log stop ---\r\n");

    /*! Flush data in to the flash memory */
    fflush(fp);

    /*! Close the file once the writing is done */
    fclose(fp);
    fclose(log_ptr);

    /*! Close all the active communication */
    close_interfaces(intf);

    return rslt;
}

static void parse_swim(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    int8_t rslt;
    uint64_t timestamp = *callback_info->time_stamp; /*! Store the last timestamp */
    struct bhy2_dev bhy2; /*! Device structure*/

    if (callback_info->data_size != 15) /*! Check for a valid payload size. Includes sensor ID */
    {
        return;
    }

    rslt = bhy2_swim_parse_data(callback_info->data_ptr, &data_out);
    print_api_error(rslt, &bhy2);

    timestamp = timestamp * 15625; /*! Timestamp is now in nanoseconds */
    seconds = (uint32_t)(timestamp / UINT64_C(1000000000));
    nano_seconds = (uint32_t)(timestamp - (seconds * UINT64_C(1000000000)));
    printf("SID: %u; T: %lu.%09lu; %d, %d, %d, %d, %d, %d, %d\r\n",
           callback_info->sensor_id,
           seconds,
           nano_seconds,
           data_out.total_distance,
           data_out.length_count,
           data_out.lengths_freestyle,
           data_out.lengths_breaststroke,
           data_out.lengths_butterfly,
           data_out.lengths_backstroke,
           data_out.stroke_count);

    /*! To log the data in the flash page of APP3.0 board */
    fprintf(fp,
            "SID: %d, T: %lu.%09lu, %d, %d, %d, %d, %d, %d, %d\n",
            CUSTOM_SENSOR_ID,
            seconds,
            nano_seconds,
            data_out.total_distance,
            data_out.length_count,
            data_out.lengths_freestyle,
            data_out.lengths_breaststroke,
            data_out.lengths_butterfly,
            data_out.lengths_backstroke,
            data_out.stroke_count);

    /*! Flush data in to the flash memory */
    fflush(fp);
}

static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
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

    switch (meta_event_type)
    {
        case BHY2_META_EVENT_FLUSH_COMPLETE:
            printf("%s Flush complete for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
            printf("%s Sample rate changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_POWER_MODE_CHANGED:
            printf("%s Power mode changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_ALGORITHM_EVENTS:
            printf("%s Algorithm event\r\n", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_STATUS:
            printf("%s Accuracy for sensor id %u changed to %u\r\n", event_text, byte1, byte2);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
            printf("%s BSX event (do steps main)\r\n", event_text);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
            printf("%s BSX event (do steps calib)\r\n", event_text);
            break;
        case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            printf("%s BSX event (get output signal)\r\n", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_ERROR:
            printf("%s Sensor id %u reported error 0x%02X\r\n", event_text, byte1, byte2);
            break;
        case BHY2_META_EVENT_FIFO_OVERFLOW:
            printf("%s FIFO overflow\r\n", event_text);
            break;
        case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
            printf("%s Dynamic range changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_FIFO_WATERMARK:
            printf("%s FIFO watermark reached\r\n", event_text);
            break;
        case BHY2_META_EVENT_INITIALIZED:
            printf("%s Firmware initialized. Firmware version %u\r\n", event_text, ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHY2_META_TRANSFER_CAUSE:
            printf("%s Transfer cause for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_SENSOR_FRAMEWORK:
            printf("%s Sensor framework event for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_RESET:
            printf("%s Reset event\r\n", event_text);
            break;
        case BHY2_META_EVENT_SPACER:
            break;
        default:
            printf("%s Unknown meta event with id: %u\r\n", event_text, meta_event_type);
            break;
    }
}

static void print_api_error(int8_t rslt, struct bhy2_dev *dev)
{
    if (rslt != BHY2_OK)
    {
        printf("%s\r\n", get_api_error(rslt));
        if ((rslt == BHY2_E_IO) && (dev != NULL))
        {
            printf("%s\r\n", get_coines_error(dev->hif.intf_rslt));
            dev->hif.intf_rslt = BHY2_INTF_RET_SUCCESS;
        }

        exit(0);
    }
}

/*!
 * @brief Uploading the firmware to the sensor RAM/FLASH
 * @param[in] dev Firmware data available here
 *
 * @return  rslt execution result
 *
 */
static int8_t upload_firmware(const char *filepath, struct bhy2_dev *bhy2)
{

    FILE *fw_file;
    struct stat st;
    uint8_t firmware_chunk[BHY2_RD_WR_LEN];
    uint32_t len;
    int8_t rslt = BHY2_OK;
    uint8_t boot_status;
    uint32_t start_time_ms;

#ifdef PC
    uint8_t progress = 0, new_progress = 0;
#endif
    bhy2_get_boot_status(&boot_status, bhy2);
    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        if ((boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) || (boot_status & BHY2_BST_FLASH_VERIFY_DONE))
        {
            printf ("Seems like a firmware is running. Please reset the BHI260/BHA260 before uploading firmware\r\n");

            return false;
        }

#ifdef PC
        fw_file = fopen(filepath, "rb"); /* Without the b, the file is read incorrectly */
#else
        fw_file = fopen(filepath, "r");
#endif

        if (!fw_file)
        {
            printf ("Cannot open file: %s\r\n", filepath);

            return false;
        }

        stat(filepath, &st);
        len = st.st_size;

        /* 256 KB */
        if (len > 262144)
        {
#ifdef BHY2_FLASH_BUILD
        	printf ("Invalid FLASH Size of %lu bytes\r\n", len);
#else
        	printf ("Invalid RAM Size of %lu bytes\r\n", len);
#endif
            return false;
        }

#ifdef BHY2_FLASH_BUILD
        printf ("Uploading %lu bytes of firmware to FLASH\r\n", len);
#else
        printf ("Uploading %lu bytes of firmware to RAM\r\n", len);
#endif

        start_time_ms = coines_get_millis();
        uint32_t incr = BHY2_RD_WR_LEN;
        if ((incr % 4) != 0)
        {
            incr = BHY2_ROUND_WORD_LOWER(incr);
        }

        for (uint32_t i = 0; (i < len) && (rslt == BHY2_OK); i += incr)
        {
            if (incr > (len - i)) /* Last payload */
            {
                incr = len - i;
                if ((incr % 4) != 0) /* Round off to higher 4 bytes */
                {
                    incr = BHY2_ROUND_WORD_HIGHER(incr);
                }
            }

            fread(firmware_chunk, 1, incr, fw_file);

#ifdef BHY2_FLASH_BUILD
            rslt = bhy2_upload_firmware_to_flash_partly(firmware_chunk, i, incr, bhy2);
#else
            rslt = bhy2_upload_firmware_to_ram_partly(firmware_chunk, len, i, incr, bhy2);
#endif

#ifdef PC
            progress = (float)(i + incr) / (float)len * 100.0f;
            if (progress != new_progress)
            {
                printf ("Completed %u %%\r", progress);
                new_progress = progress;
            }

#endif
        }

        printf ("Firmware upload took %.2f seconds\r\n", (float )(coines_get_millis() - start_time_ms) / 1000.0f);
        fclose(fw_file);

        if (rslt != BHY2_OK)
        {
            printf ("Firmware upload failed. Returned with error code: %d. %s\r\n", rslt, get_api_error(rslt));

            return false;
        }
    }
    else
    {
        printf ("Host interface is not ready\r\n");

        return false;
    }

#ifdef BHY2_FLASH_BUILD
    printf ("Uploading firmware to FLASH successful\r\n");
#else
    printf ("Uploading firmware to RAM successful\r\n");
#endif

    return true;

}
