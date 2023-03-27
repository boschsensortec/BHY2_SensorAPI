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
 * @file    euler.c
 * @brief   Euler data stream example for the BHI260/BHA260
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "bhy2.h"
#include "bhy2_parse.h"
#include "common.h"

/*#define UPLOAD_FIRMWARE_TO_FLASH */

#ifdef UPLOAD_FIRMWARE_TO_FLASH
#include "bhi260ap/BHI260AP_aux_BMM150-flash.fw.h"
#else
#include "bhi260ap/BHI260AP_aux_BMM150.fw.h"
#endif

#define WORK_BUFFER_SIZE  2048

#define EULER_SENSOR_ID   BHY2_SENSOR_ID_ORI_WU

static void parse_euler(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
static void print_api_error(int8_t rslt, struct bhy2_dev *dev);
static void upload_firmware(uint8_t boot_stat, struct bhy2_dev *dev);

enum bhy2_intf intf;

int main(void)
{
    uint8_t product_id = 0;
    uint16_t version = 0;
    int8_t rslt;
    struct bhy2_dev bhy2;
    uint8_t work_buffer[WORK_BUFFER_SIZE];
    uint8_t hintr_ctrl, hif_ctrl, boot_status;
    uint8_t accuracy; /* Accuracy is reported as a meta event. It is being printed alongside the data */

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

    /* Check for a valid product ID */
    if (product_id != BHY2_PRODUCT_ID)
    {
        printf("Product ID read %X. Expected %X\r\n", product_id, BHY2_PRODUCT_ID);
    }
    else
    {
        printf("BHI260/BHA260 found. Product ID read %X\r\n", product_id);
    }

    /* Check the interrupt pin and FIFO configurations. Disable status and debug */
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

    /* Configure the host interface */
    hif_ctrl = 0;
    rslt = bhy2_set_host_intf_ctrl(hif_ctrl, &bhy2);
    print_api_error(rslt, &bhy2);

    /* Check if the sensor is ready to load firmware */
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

        rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, (void*)&accuracy, &bhy2);
        print_api_error(rslt, &bhy2);
        rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event, (void*)&accuracy, &bhy2);
        print_api_error(rslt, &bhy2);
        rslt = bhy2_register_fifo_parse_callback(EULER_SENSOR_ID, parse_euler, (void*)&accuracy, &bhy2);
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

    /* Update the callback table to enable parsing of sensor data */
    rslt = bhy2_update_virtual_sensor_list(&bhy2);
    print_api_error(rslt, &bhy2);

    float sample_rate = 100.0; /* Read out data measured at 100Hz */
    uint32_t report_latency_ms = 0; /* Report immediately */
    rslt = bhy2_set_virt_sensor_cfg(EULER_SENSOR_ID, sample_rate, report_latency_ms, &bhy2);
    print_api_error(rslt, &bhy2);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(EULER_SENSOR_ID), sample_rate);

    while (rslt == BHY2_OK)
    {
        if (get_interrupt_status())
        {
            /* Data from the FIFO is read and the relevant callbacks if registered are called */
            rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy2);
            print_api_error(rslt, &bhy2);
        }
    }

    close_interfaces(intf);

    return rslt;
}

static void parse_euler(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    struct bhy2_data_orientation data;
    uint32_t s, ns;
    uint8_t *accuracy = (uint8_t*)callback_ref;
    if (callback_info->data_size != 7) /* Check for a valid payload size. Includes sensor ID */
    {
        return;
    }

    bhy2_parse_orientation(callback_info->data_ptr, &data);

    uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */

    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

    if (accuracy)
    {
        printf("SID: %u; T: %u.%09u; h: %f, p: %f, r: %f; acc: %u\r\n",
               callback_info->sensor_id,
               s,
               ns,
               data.heading * 360.0f / 32768.0f,
               data.pitch * 360.0f / 32768.0f,
               data.roll * 360.0f / 32768.0f,
               *accuracy);
    }
    else
    {
        printf("SID: %u; T: %u.%09u; h: %f, p: %f, r: %f\r\n",
               callback_info->sensor_id,
               s,
               ns,
               data.heading * 360.0f / 32768.0f,
               data.pitch * 360.0f / 32768.0f,
               data.roll * 360.0f / 32768.0f);
    }
}

static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    uint8_t *accuracy = (uint8_t*)callback_ref;
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
            if (accuracy)
            {
                *accuracy = byte2;
            }

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
