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
 * @file    swim.c
 * @brief   SWIM data stream
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "bhy2.h"
#include "bhy2_parse.h"
#include "common.h"
#include "bhy2_swim.h"

/*! Enable this macro for bhy2 flash build */
/*#define UPLOAD_FIRMWARE_TO_FLASH */

#ifdef UPLOAD_FIRMWARE_TO_FLASH
#include "bhi260ap_swim/BHI260AP_SWIM-flash.fw.h"
#else
#include "bhi260ap_swim/BHI260AP_SWIM.fw.h"
#endif

#define WORK_BUFFER_SIZE      2048

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
static void upload_firmware(uint8_t boot_stat, struct bhy2_dev *dev);

/*! File pointer to store the log */
FILE *fp;

/*! Device structure */
struct bhy2_dev bhy2;

/*! Structure to ge the output from the algorithm for logging */
struct bhy2_swim_algo_output data_out;

/*! Parameter for time in second and nano seconds */
uint32_t seconds, nano_seconds;

/*! Callback for button 1 interrupt */
static void button1CB(uint32_t param1, uint32_t param2);

/*! SWIM configuration parameter structure which is
 * in sync with the parameter page of swim sensor 0x0B00 */
bhy2_swim_config_param_t bhy2_swim_configuration;

/*! Swim sensor can be disabled by pressing T1 button
 * which uses this global flag to disable the sensor */
uint8_t swim_data_en = SWIM_START_STREAMING;

enum bhy2_intf intf;

int main(void)
{
    uint8_t product_id = 0;
    uint16_t version = 0;
    int8_t rslt;

    uint8_t work_buffer[WORK_BUFFER_SIZE];

    uint8_t hintr_ctrl, hif_ctrl, boot_status;

    struct bhy2_virt_sensor_conf sensor_conf;

    struct bhy2_sensor_info bhy2_sensor_info_t;

    /*! Selecting the SPI interface for sensor communication */
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

    /*! APP3.0 Board T1 button interrupt enabling */
    coines_set_pin_config(COINES_APP30_BUTTON_1, COINES_PIN_DIRECTION_IN, COINES_PIN_VALUE_HIGH);
    coines_attach_interrupt(COINES_APP30_BUTTON_1, button1CB, COINES_PIN_INTERRUPT_FALLING_EDGE);

    rslt = bhy2_soft_reset(&bhy2);
    print_api_error(rslt, &bhy2);

    rslt = bhy2_get_product_id(&product_id, &bhy2);
    print_api_error(rslt, &bhy2);

    /*! remove the existing log */
    remove("swim_logs.csv");

    /*! File open for data logging */
    fp = fopen("swim_logs.csv", "w");

    /*! Check for a valid product ID */
    if (product_id != BHY2_PRODUCT_ID)
    {
        printf("Product ID read %X. Expected %X\r\n", product_id, BHY2_PRODUCT_ID);
    }
    else
    {
        printf("BHI260/BHA260 found. Product ID read %X\r\n", product_id);
    }

    /*! Check the interrupt pin and FIFO configurations. Disable status and debug */
    hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;

    rslt = bhy2_set_host_interrupt_ctrl(hintr_ctrl, &bhy2);
    print_api_error(rslt, &bhy2);
    rslt = bhy2_get_host_interrupt_ctrl(&hintr_ctrl, &bhy2);
    print_api_error(rslt, &bhy2);

    printf("Host interrupt control\r\n");
    printf("    Wake up FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
    printf("    Non wake up FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
    printf("    Status FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
    printf("    Debugging %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
    printf("    Fault %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
    printf("    Interrupt is %s.\r\n", (hintr_ctrl & BHY2_ICTL_ACTIVE_LOW) ? "active low" : "active high");
    printf("    Interrupt is %s triggered.\r\n", (hintr_ctrl & BHY2_ICTL_EDGE) ? "pulse" : "level");
    printf("    Interrupt pin drive is %s.\r\n", (hintr_ctrl & BHY2_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");

    /*! Configure the host interface */
    hif_ctrl = 0;
    rslt = bhy2_set_host_intf_ctrl(hif_ctrl, &bhy2);
    print_api_error(rslt, &bhy2);

    /*! Check if the sensor is ready to load firmware */
    rslt = bhy2_get_boot_status(&boot_status, &bhy2);
    print_api_error(rslt, &bhy2);

    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        upload_firmware(boot_status, &bhy2);

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
    bhy2_swim_configuration.update_swim_config = BHY2_SWIM_ENABLE_CONFIG; /*! Possible values> SWIM_ENABLE_CONFIG
                                                                           * SWIM_DISABLE_CONFIG */
    bhy2_swim_configuration.dev_on_left_hand = BHY2_SWIM_DEVICE_ON_RIGHT_HAND; /*! Possible values>
                                                                                * SWIM_DEVICE_ON_LEFT_HAND
                                                                                * SWIM_DEVICE_ON_RIGHT_HAND */
    bhy2_swim_configuration.pool_length_integral = BHY2_SWIM_POOL_LENGTH_50M; /*! Possible values> SWIM_POOL_LENGTH_25M
                                                                               * SWIM_POOL_LENGTH_50M */
    bhy2_swim_configuration.pool_length_floating = 0; /*! Floating point support is yet to be added for variable pool
                                                       * length */

    /*! Writing the configuration to the parameter page */
    rslt = bhy2_swim_set_config(&bhy2_swim_configuration, &bhy2);
    print_api_error(rslt, &bhy2);

    float sample_rate = 100.0; /*! Read out data measured at 100Hz */
    uint32_t report_latency_ms = 0; /*! Report immediately */

    /*! Setting the Sampling frequency and latency time */
    rslt = bhy2_set_virt_sensor_cfg(CUSTOM_SENSOR_ID, sample_rate, report_latency_ms, &bhy2);
    print_api_error(rslt, &bhy2);

    rslt = bhy2_get_virt_sensor_cfg(CUSTOM_SENSOR_ID, &sensor_conf, &bhy2);
    printf("Swim sensor ID=%d, sensitivity=%d, rate=%.2fHz,latency=%ld\r\n",
           CUSTOM_SENSOR_ID,
           sensor_conf.sensitivity,
           sensor_conf.sample_rate,
           sensor_conf.latency);
    fprintf(fp,
            "Swim sensor ID=%d, sensitivity=%d, rate=%.2fHz,latency=%ld\r\n",
            CUSTOM_SENSOR_ID,
            sensor_conf.sensitivity,
            sensor_conf.sample_rate,
            sensor_conf.latency);

    printf("--- Swim log start ---\r\n");
    fprintf(fp, "--- Swim log start ---\r\n");

    /*! Data from the FIFO is read and the relevant callbacks if registered are called */
    while (rslt == BHY2_OK && swim_data_en != SWIM_STOP_STREAMING)
    {
        if (get_interrupt_status())
        {
            /*! Data from the FIFO is read and the relevant callbacks if registered are called */
            rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy2);
            print_api_error(rslt, &bhy2);
        }
    }

    /*! Get configuration after logging */
    rslt = bhy2_swim_get_config(&bhy2_swim_configuration, &bhy2);
    print_api_error(rslt, &bhy2);

    printf("Swim Config for above data log: pool length = %dm, Device on left hand%d\r\n",
           bhy2_swim_configuration.pool_length_integral,
           bhy2_swim_configuration.dev_on_left_hand);
    fprintf(fp,
            "Swim Config for above data log[%d] Device on left hand%d pool length = %d.%d m \r\n",
            bhy2_swim_configuration.update_swim_config,
            bhy2_swim_configuration.dev_on_left_hand,
            bhy2_swim_configuration.pool_length_integral,
            bhy2_swim_configuration.pool_length_floating);

    printf("--- Swim log stop ---\r\n");
    fprintf(fp, "--- Swim log stop ---\r\n");

    /*! Flush data in to the flash memory */
    fflush(fp);

    /*! Close the file once the writing is done */
    fclose(fp);

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
        printf(" ERRORVAL\n");

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
            "SID: %d, T: %lu.%09lu, %d, %d, %d, %d, %d, %d, %d\r\n",
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
static void upload_firmware(uint8_t boot_stat, struct bhy2_dev *dev)
{
    uint8_t sensor_error;
    int8_t temp_rslt;
    int8_t rslt = BHY2_OK;

#ifdef UPLOAD_FIRMWARE_TO_FLASH
    if (boot_stat & BHY2_BST_FLASH_DETECTED)
    {
        uint32_t start_addr = BHY2_FLASH_SECTOR_START_ADDR;
        uint32_t end_addr = start_addr + sizeof(bhy2_firmware_image);
        printf("Flash detected. Erasing flash to upload firmware\r\n");

        rslt = bhy2_erase_flash(start_addr, end_addr, dev);
        print_api_error(rslt, dev);
    }
    else
    {
        printf("Flash not detected\r\n");

        rslt = BHY2_E_IO;
        print_api_error(rslt, dev);
    }

    printf("Loading firmware into FLASH.\r\n");
    rslt = bhy2_upload_firmware_to_flash(bhy2_firmware_image, sizeof(bhy2_firmware_image), dev);
#else
    printf("Loading firmware into RAM.\r\n");
    rslt = bhy2_upload_firmware_to_ram(bhy2_firmware_image, sizeof(bhy2_firmware_image), dev);
#endif
    temp_rslt = bhy2_get_error_value(&sensor_error, dev);
    if (sensor_error)
    {
        printf("%s\r\n", get_sensor_error_text(sensor_error));
    }

    print_api_error(rslt, dev);
    print_api_error(temp_rslt, dev);

#ifdef UPLOAD_FIRMWARE_TO_FLASH
    printf("Booting from FLASH.\r\n");
    rslt = bhy2_boot_from_flash(dev);
#else
    printf("Booting from RAM.\r\n");
    rslt = bhy2_boot_from_ram(dev);
#endif

    temp_rslt = bhy2_get_error_value(&sensor_error, dev);
    if (sensor_error)
    {
        printf("%s\r\n", get_sensor_error_text(sensor_error));
    }

    print_api_error(rslt, dev);
    print_api_error(temp_rslt, dev);
}

/*! Callback for button 1 event */
void button1CB(uint32_t param1, uint32_t param2)
{
    (void)param1;
    (void)param2;

    /*! Disable sensor */
    float sample_rate = 0.0;
    uint32_t report_latency_ms = 0;
    int8_t rslt;

    /* Disable the swim sensor */
    rslt = bhy2_set_virt_sensor_cfg(CUSTOM_SENSOR_ID, sample_rate, report_latency_ms, &bhy2);
    print_api_error(rslt, &bhy2);

    /* Set the flag to disable the sensor streaming and exit from the loop */
    swim_data_en = SWIM_STOP_STREAMING;
}
