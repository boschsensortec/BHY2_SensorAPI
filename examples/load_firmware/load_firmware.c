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
 * @file    load_firmware.c
 * @brief   Example to load firmware for the BHI260/BHA260
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

#include "bhi260ap/BHI260AP-flash.fw.h"
#else
#include "bhi260ap/BHI260AP.fw.h"
#endif

static void print_api_error(int8_t rslt, struct bhy2_dev *dev);
static int8_t upload_firmware(struct bhy2_dev *dev);

enum bhy2_intf intf;

/*
 * For use-cases where the whole firmware cannot be accessed
 * through single memory mapped location, for instance, insufficient
 * memory to hold the entire firmware on the host's RAM or Flash and
 * the firmware is stored on external memory like and SD card,
 * firmware updated over USB, BLE or another medium, the following
 * examples shows how to use the bhy2_upload_firmware_to_flash_partly
 * and bhy2_upload_firmware_to_ram_partly APIs
 */

int main(void)
{
    uint8_t product_id = 0;
    uint16_t version = 0;
    int8_t rslt;
    struct bhy2_dev bhy2;

    uint8_t hintr_ctrl, hif_ctrl, boot_status;

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

    /* Configure the host interface */
    hif_ctrl = 0;
    rslt = bhy2_set_host_intf_ctrl(hif_ctrl, &bhy2);
    print_api_error(rslt, &bhy2);

    /* Check if the sensor is ready to load firmware */
    rslt = bhy2_get_boot_status(&boot_status, &bhy2);
    print_api_error(rslt, &bhy2);

    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        uint8_t sensor_error;
        int8_t temp_rslt;
        printf("Loading firmware.\r\n");

        /* If loading firmware to flash, erase the relevant section */
#ifdef UPLOAD_FIRMWARE_TO_FLASH
        if (boot_status & BHY2_BST_FLASH_DETECTED)
        {
            uint32_t start_addr = BHY2_FLASH_SECTOR_START_ADDR;
            uint32_t end_addr = start_addr + sizeof(bhy2_firmware_image);
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

        rslt = upload_firmware(&bhy2);
        temp_rslt = bhy2_get_error_value(&sensor_error, &bhy2);
        if (sensor_error)
        {
            printf("%s\r\n", get_sensor_error_text(sensor_error));
        }

        print_api_error(rslt, &bhy2);
        print_api_error(temp_rslt, &bhy2);

#ifdef UPLOAD_FIRMWARE_TO_FLASH
        printf("Booting from Flash.\r\n");
        rslt = bhy2_boot_from_flash(&bhy2);
#else
        printf("Booting from RAM.\r\n");
        rslt = bhy2_boot_from_ram(&bhy2);
#endif

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
    }
    else
    {
        printf("Host interface not ready. Exiting\r\n");

        close_interfaces(intf);

        return 0;
    }

    close_interfaces(intf);

    return rslt;
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

static int8_t upload_firmware(struct bhy2_dev *dev)
{
    uint32_t incr = 256; /* Max command packet size */
    uint32_t len = sizeof(bhy2_firmware_image);
    int8_t rslt = BHY2_OK;

    if ((incr % 4) != 0) /* Round off to higher 4 bytes */
    {
        incr = ((incr >> 2) + 1) << 2;
    }

    for (uint32_t i = 0; (i < len) && (rslt == BHY2_OK); i += incr)
    {
        if (incr > (len - i)) /* If last payload */
        {
            incr = len - i;
            if ((incr % 4) != 0) /* Round off to higher 4 bytes */
            {
                incr = ((incr >> 2) + 1) << 2;
            }
        }

#ifdef UPLOAD_FIRMWARE_TO_FLASH
        rslt = bhy2_upload_firmware_to_flash_partly(&bhy2_firmware_image[i], i, incr, dev);
#else
        rslt = bhy2_upload_firmware_to_ram_partly(&bhy2_firmware_image[i], len, i, incr, dev);
#endif

        printf("%.2f%% complete\r", (float)(i + incr) / (float)len * 100.0f);
    }

    printf("\n");

    return rslt;
}
