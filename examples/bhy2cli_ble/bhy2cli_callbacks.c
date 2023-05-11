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
 * @file    bhy2cli_callbacks.c
 * @brief   Source file for the command line utility callbacks
 *
 */

#define BHY2CLI_VER_MAJOR       "0"
#define BHY2CLI_VER_MINOR       "4"
#define BHY2CLI_VER_BUGFIX      "9"

#ifdef __STDC_ALLOC_LIB__
#define __STDC_WANT_LIB_EXT2__  1
#else
#define _POSIX_C_SOURCE         200809L
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>
#include <ctype.h>
#include <sys/stat.h>
#include <math.h>
#if defined(PC)
#include <dirent.h>
#endif

#include "coines.h"
#include "bhy2cli_callbacks.h"
#include "common_callbacks.h"
#include "common.h"
#include "parse.h"
#include "verbose.h"
#include "dinject.h"
#include "post_mortem.h"

#define BHY2_ASSERT(x)             assert_rslt = x; if (assert_rslt) check_bhy2_api(__LINE__, __FUNCTION__, assert_rslt)

#define BHY2CLI_MAX_STRING_LENGTH  UINT16_C(32)

/* Contains all parameters of the of custom virtual sensors
 * required for parsing */
typedef struct BHY2_PACKED custom_driver_information
{
    char sensor_name[BHY2CLI_MAX_STRING_LENGTH];
    uint16_t sensor_payload : 7;
    uint16_t sensor_id : 8;
    uint16_t is_registered : 1;
    char output_formats[BHY2CLI_MAX_STRING_LENGTH];
} custom_driver_information_t;

/* Contains information about klio capabilities and current state, as well as runtime configuration */
static struct
{
    uint8_t max_patterns;
    uint8_t max_pattern_blob_size;
    uint8_t auto_load_pattern_write_index;
    uint8_t auto_load_pattern;
}
klio_vars;
static int8_t assert_rslt;
static uint8_t fifo_buffer[2048];
static bool sensors_active[256] = { false };
static bool first_run = true;

/* Added to report the final swim data to mobile APP when swim is disabled */
static struct bhy2_swim_algo_output swim_data;

/* Static global table that contains the payloads of present custom virtual sensors, derived by a parameter read */
static custom_driver_information_t custom_driver_information[(BHY2_SENSOR_ID_CUSTOM_END - BHY2_SENSOR_ID_CUSTOM_START) +
                                                             1];
static bool klio_enabled = false;

struct data_inject dinject;

/*!
 * @brief To convert string input to int
 */
uint32_t string_to_int(const char *str)
{
    char int_str[32] = { 0 };

    strncpy(int_str, str, strlen(str));
    int_str[strlen(str)] = '\0';

    return (uint32_t)strtol(int_str, NULL, 0);
}

static cli_callback_table_t bhy2_cli_callbacks[] = {
    { 0, "", 0, NULL, NULL }, /* Empty characters creates a new line */
    { 'h', "help", 0, help_callback, help_help }, /* Print all available commands */
    { 0, "version", 0, version_callback, version_help }, /* Prints the HW, SW versions and build date*/
    { 'v', "verb", 1, verbose_callback, verb_help }, /* Change verbose of received outputs */
    { 'b', "ramb", 1, ramb_callback, ramb_help }, /* Reset, Load firmware to RAM and boot */
    { 'd', "flb", 1, flb_callback, flb_help }, /* Reset, Load firmware to Flash and boot */
    { 'n', "reset", 0, reset_callback, reset_help }, /* Trigger a soft reset to the sensor */
    { 'a', "addse", 1, addse_callback, addse_help }, /* Add a custom sensor */
    { 'g', "boot", 1, boot_callback, boot_help }, /* Boot from RAM or Flash */
    { 'c', "actse", 1, actse_callback, actse_help }, /* Activate/De-activate a sensor */
    { 0, "schema", 0, schema_callback, schema_help }, /* Get schema information of the loaded sensors */
    { 0, "hexse", 1, hexse_callback, hexse_help }, /* Stream sensor data in hex */
    { 0, "dactse", 0, dactse_callback, dactse_help }, /* Deactivate all the active sensors */
    { 0, "lsactse", 0, lsactse_callback, lsactse_help }, /* List all the active sensors */
    { 0, "dmode", 1, dmode_callback, dmode_help }, /* Switch to Data Injection mode */
    { 0, "dinject", 1, dinject_callback, dinject_help }, /* Compute virtual sensor output from raw IMU data */
    { 'e', "erase", 0, erase_callback, erase_help }, /* Erase the external Flash */
    { 0, "efd", 0, efd_callback, efd_help }, /* Erase the Flash descriptor */
    { 'r', "rd", 1, rd_callback, rd_help }, /* Read registers */
    { 'u', "ram", 1, ram_callback, ram_help }, /* Upload firmware to RAM */
    { 'f', "fl", 1, fl_callback, fl_help }, /* Upload firmware to Flash */
    { 'w', "wr", 1, wr_callback, wr_help }, /* Write registers */
    { 'i', "info", 0, info_callback, info_help }, /* Get information of the state of the device and loaded sensors */
    { 's', "rdp", 1, rdp_callback, rdp_help }, /* Read a parameter */
    { 't', "wrp", 1, wrp_callback, wrp_help }, /* Write a parameter */
    { 'p', "physeninfo", 1, physeninfo_callback, physeninfo_help }, /* Read Physical Sensor Information */
    { 'm', "postm", 1, pm_callback, pm_help }, /* Get Post Mortem Data */
    { 0, "logse", 1, logse_callback, logse_help }, /* Log sensor data in binary */
    { 0, "attlog", 1, attlog_callback, attlog_help }, /* Attach a log file for logging */
    { 0, "detlog", 1, detlog_callback, detlog_help }, /* Detach a log file for logging */
    { 0, "kstatus", 0, kstatus_callback, kstatus_help }, /* Get Klio status */
    { 0, "ksetstate", 4, ksetstate_callback, ksetstate_help }, /* Set Klio state */
    { 0, "kgetstate", 0, kgetstate_callback, kgetstate_help }, /* Get Klio state */
    { 0, "kldpatt", 2, kldpatt_callback, kldpatt_help }, /* Load Klio pattern */
    { 0, "kenpatt", 1, kenpatt_callback, kenpatt_help }, /* Enable Klio pattern */
    { 0, "kdispatt", 1, kdispatt_callback, kdispatt_help }, /* Disable Klio pattern */
    { 0, "kdisapatt", 1, kdisapatt_callback, kdisapatt_help }, /* Disable Klio adaptive pattern */
    { 0, "kswpatt", 1, kswpatt_callback, kswpatt_help }, /* Switch Klio pattern between left/right hand */
    { 0, "kautldpatt", 2, kautldpatt_callback, kautldpatt_help }, /* Auto-load Klio patterns */
    { 0, "kgetparam", 1, kgetparam_callback, kgetparam_help }, /* Get Klio parameters */
    { 0, "ksetparam", 2, ksetparam_callback, ksetparam_help }, /* Set Klio parameters */
    { 0, "ksimscore", 2, ksimscore_callback, ksimscore_help }, /* Get Klio Similarity score */
    { 0, "kmsimscore", 2, kmsimscore_callback, kmsimscore_help }, /* Get Multiple Klio Similarity score */
    { 0, "swim", 3, swim_callback, swim_help }, /* Configure the Swim recognition */
    { 0, "swimver", 0, swimver_callback, swimver_help }, /* Get the Swim Version */
    { 0, "swimgetfreq", 0, swimgetfreq_callback, swimgetfreq_help }, /* Get the Swim frequency */
    { 0, "swimsetfreq", 2, swimsetfreq_callback, swimsetfreq_help }, /* Set the Swim frequency */
    { 0, "swimgetaxes", 0, swimgetaxes_callback, swimgetaxes_help }, /* Get the Swim orientation sensor */
    { 0, "swimsetaxes", 1, swimsetaxes_callback, swimsetaxes_help }, /* Set the Swim orientation sensor */
    { 0, "mtapen", 1, mtapen_callback, mtapen_help }, /* Enable/Disable Multi Tap Sensor */
    { 0, "mtapinfo", 0, mtapinfo_callback, mtapinfo_help }, /* Get Multi Tap Sensor Info */
    { 0, "mtapsetcnfg", 3, mtapsetcnfg_callback, mtapsetcnfg_help }, /* Set the Multi Tap Configuration */
    { 0, "mtapgetcnfg", 0, mtapgetcnfg_callback, mtapgetcnfg_help }, /* Get the Multi Tap Configuration */
    { 0, "accsetfoc", 3, accsetfoc_callback, accsetfoc_help }, /* Set the Accelerometer Fast Offset Calibration */
    { 0, "accgetfoc", 0, accgetfoc_callback, accgetfoc_help }, /* Get the Accelerometer Fast Offset Calibration */
    { 0, "accsetpwm", 1, accsetpwm_callback, accsetpwm_help }, /* Set the Accelerometer Power Mode */
    { 0, "accgetpwm", 0, accgetpwm_callback, accgetpwm_help }, /* Get the Accelerometer Power Mode */
    { 0, "gyrosetfoc", 3, gyrosetfoc_callback, gyrosetfoc_help }, /* Set the Gyroscope Fast Offset Calibration */
    { 0, "gyrogetfoc", 0, gyrogetfoc_callback, gyrogetfoc_help }, /* Get the Gyroscope Fast Offset Calibration */
    { 0, "gyrosetois", 1, gyrosetois_callback, gyrosetois_help }, /* Set the Gyroscope OIS Mode*/
    { 0, "gyrogetois", 0, gyrogetois_callback, gyrogetois_help }, /* Get the Gyroscope OIS Mode*/
    { 0, "gyrosetfs", 1, gyrosetfs_callback, gyrosetfs_help }, /* Set the Gyroscope Fast Startup Mode */
    { 0, "gyrogetfs", 0, gyrogetfs_callback, gyrogetfs_help }, /* Get the Gyroscope Fast Startup Mode*/
    { 0, "gyrosetcrt", 0, gyrosetcrt_callback, gyrosetcrt_help }, /* Set the Gyroscope CRT state*/
    { 0, "gyrogetcrt", 0, gyrogetcrt_callback, gyrogetcrt_help }, /* Get the Gyroscope CRT status*/
    { 0, "gyrosetpwm", 1, gyrosetpwm_callback, gyrosetpwm_help }, /* Set the Gyroscope Power Mode */
    { 0, "gyrogetpwm", 0, gyrogetpwm_callback, gyrogetpwm_help }, /* Get the Gyroscope Power Mode */
    { 0, "gyrosettat", 1, gyrosettat_callback, gyrosettat_help }, /* Set the Gyroscope Timer Auto Trim state*/
    { 0, "gyrogettat", 0, gyrogettat_callback, gyrogettat_help }, /* Get the Gyroscope Timer Auto Trim status*/
    { 0, "wwwsetcnfg", 8, wwwsetcnfg_callback, wwwsetcnfg_help }, /* Set the Wrist Wear Wakeup Configuration */
    { 0, "wwwgetcnfg", 0, wwwgetcnfg_callback, wwwgetcnfg_help }, /* Get the Wrist Wear Wakeup Configuration */
    { 0, "amsetcnfg", 1, amsetcnfg_callback, amsetcnfg_help }, /* Set the Any Motion Configuration */
    { 0, "amgetcnfg", 0, amgetcnfg_callback, amgetcnfg_help }, /* Get the Any Motion Configuration */
    { 0, "nmsetcnfg", 1, nmsetcnfg_callback, nmsetcnfg_help }, /* Set the No Motion Configuration */
    { 0, "nmgetcnfg", 0, nmgetcnfg_callback, nmgetcnfg_help }, /* Get the No Motion Configuration */
    { 0, "wgdsetcnfg", 10, wgdsetcnfg_callback, wgdsetcnfg_help }, /* Set the Wrist Gesture Detection Configuration */
    { 0, "wgdgetcnfg", 0, wgdgetcnfg_callback, wgdgetcnfg_help }, /* Get the Wrist Gesture Detection Configuration */
    { 0, "hmctrig", 0, hmctrig_callback, hmctrig_help }, /* Trigger Head Misalignment Calibration */
    { 0, "hmcsetcnfg", 4, hmcsetcnfg_callback, hmcsetcnfg_help }, /* Set Head Misalignment Configuration */
    { 0, "hmcgetcnfg", 0, hmcgetcnfg_callback, hmcgetcnfg_help }, /* Get Head Misalignment Configuration */
    { 0, "hmcsetdefcnfg", 0, hmcsetdefcnfg_callback, hmcsetdefcnfg_help }, /* Set Default Head Misalignment
                                                                            * Configuration */
    { 0, "hmcver", 0, hmcver_callback, hmcver_help }, /* Get Head Misalignment Calibrator Version */
    { 0, "hmcsetcalcorrq", 4, hmcsetcalcorrq_callback, hmcsetcalcorrq_help }, /* Set Head Misalignment Quaternion
                                                                               * Calibration Correction */
    { 0, "hmcgetcalcorrq", 0, hmcgetcalcorrq_callback, hmcgetcalcorrq_help }, /* Get Head Misalignment Quaternion
                                                                               * Calibration Correction */
    { 0, "hosetheadcorrq", 1, hosetheadcorrq_callback, hosetheadcorrq_help }, /* Get Head Orientation
                                                                                        * Quaternion Initial Head
                                                                                        * Correction */
    { 0, "hogetheadcorrq", 0, hogetheadcorrq_callback, hogetheadcorrq_help }, /* Get Head Orientation
                                                                                        * Quaternion Initial Head
                                                                                        * Correction */
    { 0, "hover", 0, hover_callback, hover_help }, /* Get Head Orientation Version */
    { 0, "hosetheadcorre", 1, hosetheadcorre_callback, hosetheadcorre_help }, /* Get Head Orientation Euler
                                                                                     * Initial Head Correction */
    { 0, "hogetheadcorre", 0, hogetheadcorre_callback, hogetheadcorre_help }, /* Get Head Orientation Euler Initial Head
                                                                               * Correction */
#ifndef PC
    { 0, "echo", 1, echo_callback, echo_help }, /* Toggle the echo setting */
    { 0, "heart", 1, heartbeat_callback, heartbeat_help }, /* Toggle the heartbeat message setting */
    { 0, "mklog", 1, mklog_callback, mklog_help }, /* Make a log file */
    { 0, "rm", 1, rm_callback, rm_help }, /* Remove a file */
    { 0, "ls", 0, ls_callback, ls_help }, /* List files */
    { 0, "wrfile", 2, wrfile_callback, wrfile_help }, /* Write content to a file */
    { 0, "rdfile", 1, rdfile_callback, rdfile_help }, /* Read content from a file */
    { 0, "slabel", 1, slabel_callback, slabel_help }, /* Write a binary label into the log file */
    { 0, "cls", 0, cls_callback, cls_help }, /* Clear screen */
    { 0, "strbuf", 1, streambuff_callback, streambuff_help }, /* Enable streaming buffer */
#endif
};

static void check_bhy2_api(unsigned int line, const char *func, int8_t val);
static void reset_hub(struct bhy2_dev *bhy2);
static bool upload_to_ram(const char *filepath, struct bhy2_dev *bhy2);
static void print_boot_status(uint8_t boot_status);
static void boot_ram(struct bhy2_dev *bhy2);
static void show_info(struct bhy2_dev *bhy2);
static void boot_flash(struct bhy2_dev *bhy2);
static bool upload_to_flash(const char *filepath, struct bhy2_dev *bhy2);
static void wr_regs(const char *payload, struct bhy2_dev *bhy2);
static void rd_regs(const char *payload, struct bhy2_dev *bhy2);
static void rd_param(const char *payload, struct bhy2_dev *bhy2);
static void wr_param(const char *payload, struct bhy2_dev *bhy2);
static void rd_phy_sensor_info(const char *payload, struct bhy2_dev *bhy2);
static void erase_flash(uint32_t end_addr, struct bhy2_dev *bhy2);
static void activate_sensor(const char *sensor_parameters, uint8_t parse_flag, struct bhy2_cli_ref *ref);
static void parse_custom_sensor_default(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
static void parse_custom_sensor(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
static void add_sensor(const char *payload, struct bhy2_cli_ref *cli_ref);
static void klio_enable(struct bhy2_dev *bhy2);
static void klio_status(struct bhy2_dev *bhy2);
static void klio_set_state(const char *arg1, const char *arg2, const char *arg3, const char *arg4,
                           struct bhy2_dev *bhy2);
static void klio_get_state(struct bhy2_dev *bhy2);
static void klio_load_pattern(const char *arg1, const char *arg2, struct bhy2_dev *bhy2);
static void klio_get_parameter(const uint8_t *arg, struct bhy2_dev *bhy2);
static void klio_set_parameter(const char *arg1, char *arg2, struct bhy2_dev *bhy2);
static void klio_similarity_score(const uint8_t *arg1, const uint8_t *arg2, struct bhy2_dev *bhy2);
static void klio_similarity_score_multiple(const char *arg1, const char *arg2, struct bhy2_dev *bhy2);
static void klio_pattern_state_operation(const uint8_t enable, const char *arg1, struct bhy2_dev *bhy2);
void parse_klio(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_klio_log(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_swim(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_acc_gyro(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_multitap(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_wrist_gesture_detect(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_air_quality(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_hmc(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_oc(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_ec(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
static void log_data(uint8_t sid, uint64_t tns, uint8_t event_size, uint8_t *event_payload, struct logbin_dev *logdev);
static void write_meta_info(struct logbin_dev *log, struct bhy2_dev *bhy2);
static void stream_hex_data(uint8_t sid, uint32_t ts, uint32_t tns, uint8_t event_size, uint8_t *event_payload);
static void schema_info(struct bhy2_dev *bhy2);

cli_callback_table_t *bhy2_get_cli_callbacks(void)
{
    return bhy2_cli_callbacks;
}

uint8_t bhy2_get_n_cli_callbacks(void)
{
    return sizeof(bhy2_cli_callbacks) / sizeof(cli_callback_table_t);
}

void bhy2_callbacks_init(struct bhy2_cli_ref *cli_ref)
{
    struct bhy2_dev *bhy2 = &cli_ref->bhy2;
    uint8_t expected_data;

    for (uint8_t i = BHY2_SENSOR_ID_CUSTOM_START;
         i <= BHY2_SENSOR_ID_CUSTOM_END; i++)
    {
        custom_driver_information[i - BHY2_SENSOR_ID_CUSTOM_START].is_registered = 0;
        strcpy(custom_driver_information[i - BHY2_SENSOR_ID_CUSTOM_START].sensor_name, "Undefined custom sensor");
    }

    /* Print Copyright build date */
    PRINT("Copyright (c) 2023 Bosch Sensortec GmbH\r\n");
    PRINT("Version %s.%s.%s Build date: " __DATE__ "\r\n", BHY2CLI_VER_MAJOR, BHY2CLI_VER_MINOR, BHY2CLI_VER_BUGFIX);
#ifdef BHY2_USE_I2C
    BHY2_ASSERT(bhy2_init(BHY2_I2C_INTERFACE, bhy2_i2c_read, bhy2_i2c_write, bhy2_delay_us, BHY2_RD_WR_LEN, NULL,
                          bhy2));
#else
    BHY2_ASSERT(bhy2_init(BHY2_SPI_INTERFACE, bhy2_spi_read, bhy2_spi_write, bhy2_delay_us, BHY2_RD_WR_LEN, NULL,
                          bhy2));
#endif

    /* Install virtual sensor callbacks */
    bhy2_install_callbacks(&cli_ref->bhy2, &cli_ref->parse_table);

    coines_delay_msec(100); /* Wait for flash firmware to load if applicable */

    uint8_t product_id;

    BHY2_ASSERT(bhy2_get_product_id(&product_id, bhy2));
    if (product_id == BHY2_PRODUCT_ID)
    {
        uint8_t feat_status;
        BHY2_ASSERT(bhy2_get_feature_status(&feat_status, bhy2));
        INFO("Device found\r\n");
        if (feat_status & BHY2_FEAT_STATUS_OPEN_RTOS_MSK)
        {
            INFO("RTOS based firmware running\r\n");

            BHY2_ASSERT(bhy2_update_virtual_sensor_list(bhy2));
        }
    }
    else
    {
        ERROR("Device not found, Check connections and power. Product ID read 0x%x\r\n", product_id);
#ifdef PC
        exit(1);
#endif
    }

    /* Config status channel */
    BHY2_ASSERT(bhy2_set_host_intf_ctrl(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL, bhy2));
    BHY2_ASSERT(bhy2_get_host_intf_ctrl(&expected_data, bhy2));
    if (!(expected_data & BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL))
    {
        WARNING("Expected Host Interface Control (0x06) to have bit 0x%x to be set\r\n",
                BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL);
    }
}

bool bhy2_are_sensors_active(void)
{
    for (uint16_t i = 0; i < 256; i++)
    {
        if (sensors_active[i])
        {
            return true;
        }
    }

    return false;
}

void bhy2_exit(struct bhy2_cli_ref *cli_ref)
{
    for (uint16_t i = 0; i < 256; i++)
    {
        if (sensors_active[i])
        {
            bhy2_set_virt_sensor_cfg(i, 0.0f, 0, &cli_ref->bhy2);
            sensors_active[i] = false;
        }
    }

    if (cli_ref->parse_table.logdev.logfile)
    {
        fclose(cli_ref->parse_table.logdev.logfile);
        cli_ref->parse_table.logdev.logfile = NULL;
        memset(cli_ref->parse_table.logdev.logfilename, 0, sizeof(cli_ref->parse_table.logdev.logfilename));
    }
}

void bhy2_data_parse_callback(struct bhy2_cli_ref *cli_ref)
{
    if (get_interrupt_status())
    {
        BHY2_ASSERT(bhy2_get_and_process_fifo(fifo_buffer, sizeof(fifo_buffer), &cli_ref->bhy2));
    }
}

bhy2_fifo_parse_callback_t bhy2_get_callback(uint8_t sensor_id)
{
    bhy2_fifo_parse_callback_t callback = NULL;

    switch (sensor_id)
    {
        case BHY2_SENSOR_ID_ACC:
        case BHY2_SENSOR_ID_ACC_WU:
        case BHY2_SENSOR_ID_ACC_PASS:
        case BHY2_SENSOR_ID_ACC_RAW:
        case BHY2_SENSOR_ID_ACC_RAW_WU:
        case BHY2_SENSOR_ID_ACC_BIAS:
        case BHY2_SENSOR_ID_ACC_BIAS_WU:
        case BHY2_SENSOR_ID_GRA:
        case BHY2_SENSOR_ID_GRA_WU:
        case BHY2_SENSOR_ID_LACC:
        case BHY2_SENSOR_ID_LACC_WU:
        case BHY2_SENSOR_ID_MAG:
        case BHY2_SENSOR_ID_MAG_WU:
        case BHY2_SENSOR_ID_MAG_PASS:
        case BHY2_SENSOR_ID_MAG_RAW:
        case BHY2_SENSOR_ID_MAG_RAW_WU:
        case BHY2_SENSOR_ID_MAG_BIAS:
        case BHY2_SENSOR_ID_MAG_BIAS_WU:
        case BHY2_SENSOR_ID_GYRO:
        case BHY2_SENSOR_ID_GYRO_WU:
        case BHY2_SENSOR_ID_GYRO_PASS:
        case BHY2_SENSOR_ID_GYRO_RAW:
        case BHY2_SENSOR_ID_GYRO_RAW_WU:
        case BHY2_SENSOR_ID_GYRO_BIAS:
        case BHY2_SENSOR_ID_GYRO_BIAS_WU:
            callback = parse_3axis_s16;
            break;
        case BHY2_SENSOR_ID_ORI:
        case BHY2_SENSOR_ID_ORI_WU:
            callback = parse_euler;
            break;
        case BHY2_SENSOR_ID_RV:
        case BHY2_SENSOR_ID_RV_WU:
        case BHY2_SENSOR_ID_GAMERV:
        case BHY2_SENSOR_ID_GAMERV_WU:
        case BHY2_SENSOR_ID_GEORV:
        case BHY2_SENSOR_ID_GEORV_WU:
            callback = parse_quaternion;
            break;
        case BHY2_SENSOR_ID_HUM:
        case BHY2_SENSOR_ID_HUM_WU:
            callback = parse_scalar_u8;
            break;
        case BHY2_SENSOR_ID_GAS:
        case BHY2_SENSOR_ID_GAS_WU:
            callback = parse_scalar_u32;
            break;
        case BHY2_SENSOR_ID_DEVICE_ORI:
        case BHY2_SENSOR_ID_DEVICE_ORI_WU:
            callback = parse_device_ori;
            break;
        case BHY2_SENSOR_ID_TEMP:
        case BHY2_SENSOR_ID_TEMP_WU:
            callback = parse_s16_as_float;
            break;
        case BHY2_SENSOR_ID_BARO:
        case BHY2_SENSOR_ID_BARO_WU:
            callback = parse_u24_as_float;
            break;
        case BHY2_SENSOR_ID_KLIO:
            callback = parse_klio;
            break;
        case BHY2_SENSOR_ID_KLIO_LOG:
            callback = parse_klio_log;
            break;
        case BHY2_SENSOR_ID_SWIM:
            callback = parse_swim;
            break;
        case BHY2_SENSOR_ID_SI_ACCEL:
        case BHY2_SENSOR_ID_SI_GYROS:
            callback = parse_acc_gyro;
            break;
        case BHY2_SENSOR_ID_LIGHT:
        case BHY2_SENSOR_ID_LIGHT_WU:
            callback = parse_s16_as_float;
            break;
        case BHY2_SENSOR_ID_PROX:
        case BHY2_SENSOR_ID_PROX_WU:
        case BHY2_SENSOR_ID_EXCAMERA:
            callback = parse_scalar_u8;
            break;
        case BHY2_SENSOR_ID_STC:
        case BHY2_SENSOR_ID_STC_WU:
        case BHY2_SENSOR_ID_STC_LP:
        case BHY2_SENSOR_ID_STC_LP_WU:
            callback = parse_scalar_u32;
            break;
        case BHY2_SENSOR_ID_SIG:
        case BHY2_SENSOR_ID_STD:
        case BHY2_SENSOR_ID_STD_WU:
        case BHY2_SENSOR_ID_TILT_DETECTOR:
        case BHY2_SENSOR_ID_WAKE_GESTURE:
        case BHY2_SENSOR_ID_GLANCE_GESTURE:
        case BHY2_SENSOR_ID_PICKUP_GESTURE:
        case BHY2_SENSOR_ID_SIG_LP:
        case BHY2_SENSOR_ID_SIG_LP_WU:
        case BHY2_SENSOR_ID_STD_LP:
        case BHY2_SENSOR_ID_STD_LP_WU:
        case BHY2_SENSOR_ID_WRIST_TILT_GESTURE:
        case BHY2_SENSOR_ID_STATIONARY_DET:
        case BHY2_SENSOR_ID_ANY_MOTION_LP:
        case BHY2_SENSOR_ID_ANY_MOTION_LP_WU:
        case BHI3_SENSOR_ID_NO_MOTION_LP_WU:
        case BHY2_SENSOR_ID_MOTION_DET:
        case BHI3_SENSOR_ID_WRIST_WEAR_LP_WU:
            callback = parse_scalar_event;
            break;
        case BHY2_SENSOR_ID_AR:
        case BHI3_SENSOR_ID_AR_WEAR_WU:
            callback = parse_activity;
            break;
        case BHY2_SENSOR_ID_GPS:
            callback = parse_gps;
            break;
        case BHI3_SENSOR_ID_WRIST_GEST_DETECT_LP_WU:
            callback = parse_wrist_gesture_detect;
            break;
        case BHI3_SENSOR_ID_MULTI_TAP:
            callback = parse_multitap;
            break;
        case BHY2_SENSOR_ID_AIR_QUALITY:
            callback = parse_air_quality;
            break;
        case BHY2_SENSOR_ID_HEAD_ORI_MIS_ALG:
            callback = parse_hmc;
            break;
        case BHY2_SENSOR_ID_IMU_HEAD_ORI_Q:
        case BHY2_SENSOR_ID_NDOF_HEAD_ORI_Q:
            callback = parse_oc;
            break;
        case BHY2_SENSOR_ID_IMU_HEAD_ORI_E:
        case BHY2_SENSOR_ID_NDOF_HEAD_ORI_E:
            callback = parse_ec;
            break;
        default:
            callback = parse_generic;
            break;
    }

    return callback;
}

int8_t kstatus_help(void *ref)
{
    PRINT("  kstatus\r\n");
    PRINT("        = Get and reset current klio driver status\r\n");

    return CLI_OK;
}

int8_t kstatus_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s\r\n", argv[0]);
    klio_enable(&cli_ref->bhy2);
    klio_status(&cli_ref->bhy2);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t ksetstate_help(void *ref)
{
    PRINT("  ksetstate <le> <lr> <re> <rr>\r\n");
    PRINT("        = Set klio state\r\n");
    PRINT("         <le> learning enable (0/1)\r\n");
    PRINT("         <lr> learning reset (0/1)\r\n");
    PRINT("         <re> recognition enable (0/1)\r\n");
    PRINT("         <rr> recognition reset (0/1)\r\n");

    return CLI_OK;
}

int8_t ksetstate_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s %s %s %s\r\n",
         (char *)argv[0],
         (char *)argv[1],
         (char *)argv[2],
         (char *)argv[3],
         (char *)argv[4]);
    klio_enable(&cli_ref->bhy2);
    klio_set_state((char *)argv[1], (char *)argv[2], (char *)argv[3], (char *)argv[4], &cli_ref->bhy2);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t kgetstate_help(void *ref)
{
    PRINT("  kgetstate\r\n");
    PRINT("        = Get current Klio state\r\n");

    return CLI_OK;
}

int8_t kldpatt_help(void *ref)
{
    PRINT("  kldpatt <index> <pattern>\r\n");
    PRINT("        = Load a pattern/adaptive pattern for recognition. If loading an\r\n");
    PRINT("          adaptive pattern, a regular pattern must have been previously\r\n");
    PRINT("          loaded on the given index\r\n");
    PRINT("         <index> pattern index to write to\r\n");
    PRINT("         <pattern> pattern/adaptive pattern as bare hex bytestring\r\n");

    return CLI_OK;
}

int8_t kenpatt_help(void *ref)
{
    PRINT("  kenpatt <patterns>\r\n");
    PRINT("        = Enable pattern ids for recognition\r\n");
    PRINT("         <patterns> pattern indices to enable, specified as 0,1,4 etc\r\n");

    return CLI_OK;
}

int8_t kdispatt_help(void *ref)
{
    PRINT("  kdispatt <patterns>\r\n");
    PRINT("        = Disable pattern ids for recognition\r\n");
    PRINT("         <patterns> pattern indices to disable, specified as 0,1,4 etc\r\n");

    return CLI_OK;
}

int8_t kdisapatt_help(void *ref)
{
    PRINT("  kdisapatt <patterns>\r\n");
    PRINT("        = Disable pattern adaptation for given pattern ids\r\n");
    PRINT("         <patterns> pattern indices to disable, specified as 0,1,4 etc\r\n");

    return CLI_OK;
}

int8_t kswpatt_help(void *ref)
{
    PRINT("  kswpatt <patterns>\r\n");
    PRINT("        = Switch pattern between left/right hand\r\n");
    PRINT("         <patterns> pattern indices to switch, specified as 0,1,4 etc\r\n");

    return CLI_OK;
}

int8_t kautldpatt_help(void *ref)
{
    PRINT("  kautldpatt <enable> <index>\r\n");
    PRINT("        = Automatically use learnt patterns for recognition\r\n");
    PRINT("         <enable> enable or disable (1/0)\r\n");
    PRINT("         <index> pattern index to start loading into (normally 0)\r\n");

    return CLI_OK;
}

int8_t kgetparam_help(void *ref)
{
    PRINT("  kgetparam <param>\r\n");
    PRINT("        = Print klio parameter\r\n");
    PRINT("         <param> parameter id, see documentation\r\n");

    return CLI_OK;
}

int8_t kgetstate_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s\r\n", argv[0]);
    klio_enable(&cli_ref->bhy2);
    klio_get_state(&cli_ref->bhy2);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t kldpatt_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    klio_enable(&cli_ref->bhy2);
    klio_load_pattern((char *)argv[1], (char *)argv[2], &cli_ref->bhy2);
    PRINT("Loaded pattern %s for recognition\r\n", argv[1]);

    return CLI_OK;
}

int8_t kenpatt_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    klio_enable(&cli_ref->bhy2);
    klio_pattern_state_operation(KLIO_PATTERN_STATE_ENABLE, (char *)argv[1], &cli_ref->bhy2);
    PRINT("Enabled pattern %s for recognition\r\n", argv[1]);

    return CLI_OK;
}

int8_t kdispatt_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    klio_enable(&cli_ref->bhy2);
    klio_pattern_state_operation(KLIO_PATTERN_STATE_DISABLE, (char *)argv[1], &cli_ref->bhy2);
    PRINT("Disabled pattern %s from recognition\r\n", argv[1]);

    return CLI_OK;
}

int8_t kdisapatt_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    klio_enable(&cli_ref->bhy2);
    klio_pattern_state_operation(KLIO_PATTERN_STATE_AP_DISABLE, (char *)argv[1], &cli_ref->bhy2);
    PRINT("Disable adaptation for pattern %s\r\n", argv[1]);

    return CLI_OK;
}

int8_t kswpatt_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    klio_enable(&cli_ref->bhy2);
    klio_pattern_state_operation(KLIO_PATTERN_STATE_SWITCH_HAND, (char *)argv[1], &cli_ref->bhy2);
    PRINT("Switched pattern to opposite hand for pattern %s\r\n", argv[1]);

    return CLI_OK;
}

int8_t kautldpatt_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s %s\r\n", argv[0], argv[1], argv[2]);
    klio_enable(&cli_ref->bhy2);
    klio_vars.auto_load_pattern = atoi((char *)argv[1]);
    klio_vars.auto_load_pattern_write_index = atoi((char *)argv[2]);
    PRINT("Klio auto load pattern %s, starting from index %s\r\n",
          (klio_vars.auto_load_pattern == 0) ? "Disabled" : "Enabled",
          argv[2]);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t kgetparam_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    klio_enable(&cli_ref->bhy2);
    klio_get_parameter(argv[1], &cli_ref->bhy2);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t ksetparam_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s %s\r\n", argv[0], argv[1], argv[2]);
    klio_enable(&cli_ref->bhy2);
    klio_set_parameter((char *)argv[1], (char *)argv[2], &cli_ref->bhy2);
    PRINT("Set value %s for parameter id %s\r\n", argv[2], argv[1]);

    return CLI_OK;
}

int8_t ksimscore_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s\r\n", argv[0], argv[1], argv[2]);
    klio_enable(&cli_ref->bhy2);
    klio_similarity_score(argv[1], argv[2], &cli_ref->bhy2);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t kmsimscore_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s\r\n", argv[0], argv[1], argv[2]);
    klio_enable(&cli_ref->bhy2);
    klio_similarity_score_multiple((const char *)argv[1], (const char *)argv[2], &cli_ref->bhy2);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t ksetparam_help(void *ref)
{
    PRINT("  ksetparam <param> <value>\r\n");
    PRINT("        = Set klio parameter\r\n");
    PRINT("         <param> parameter id, see documentation\r\n");
    PRINT("         <value> depends on parameter id, see documentation\r\n");

    return CLI_OK;
}

int8_t ksimscore_help(void *ref)
{
    PRINT("  ksimscore <pattern1> <pattern2>\r\n");
    PRINT("        = Print similarity score for two patterns\r\n");
    PRINT("         <pattern1> first pattern as bare hex bytestring\r\n");
    PRINT("         <pattern2> second pattern as bare hex bytestring\r\n");

    return CLI_OK;
}

int8_t kmsimscore_help(void *ref)
{
    PRINT("  kmsimscore <base index> <comparison indices>\r\n");
    PRINT("        = Print similarity score for one or more stored patterns\r\n");
    PRINT("         <base index> compare the patterns in <comparison indices> with this pattern index\r\n");
    PRINT(
        "         <comparison indices> pattern indices to compare with pattern in base index, specified as 0,1,4 etc\r\n");

    return CLI_OK;
}

int8_t version_help(void *ref)
{
    PRINT("  version\r\n");
    PRINT("        = Prints the version\r\n");

    return CLI_OK;
}

int8_t version_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct coines_board_info board_info;

    INFO("Executing %s\r\n", argv[0]);

    coines_get_board_info(&board_info);

    PRINT("HW info:: Board: %u, HW ID: %X, Shuttle ID: %X, SW ID: %X\r\n",
          board_info.board,
          board_info.hardware_id,
          board_info.shuttle_id,
          board_info.software_id);
    PRINT("SW Version: %s.%s.%s\r\nBuild date: " __DATE__ "\r\n\r\n\r\n",
          BHY2CLI_VER_MAJOR,
          BHY2CLI_VER_MINOR,
          BHY2CLI_VER_BUGFIX);

    return CLI_OK;
}

int8_t help_help(void *ref)
{
    PRINT("Usage:\r\n");
    PRINT("bhy2cli [<options>]\r\n");
    PRINT("Options:\r\n");
    PRINT("  -h OR help\t= Print this usage message\r\n");

    return CLI_OK;
}

int8_t help_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s\r\n", argv[0]);

    cli_help(ref, &(cli_ref->cli_dev));

    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t info_help(void *ref)
{
    PRINT("  -i OR info\t= Show device information: Device ID,\r\n");
    PRINT("    \t  ROM version, RAM version, Power state,\r\n");
    PRINT("    \t  list of available sensors,\r\n");
    PRINT("    \t  content of Boot Status register,\r\n");
    PRINT("    \t  content of Error value register\r\n");

    return CLI_OK;
}

int8_t info_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s\r\n", argv[0]);
    show_info(&cli_ref->bhy2);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t ramb_help(void *ref)
{
    PRINT("  -b OR ramb <firmware path>\r\n");
    PRINT("    \t= Reset, upload specified firmware to RAM and boot from RAM\r\n");
    PRINT("    \t  [equivalent to using \"reset ram <firmware> boot r\" successively]\r\n");

    return CLI_OK;
}

int8_t ramb_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    reset_hub(&cli_ref->bhy2);
    if (upload_to_ram((char *)argv[1], &cli_ref->bhy2))
    {
        boot_ram(&cli_ref->bhy2);
        bhy2_get_virt_sensor_list(&cli_ref->bhy2);
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t flb_help(void *ref)
{
    PRINT("   -d OR flb <firmware path>\r\n");
    PRINT("    \t= Reset, upload specified firmware to Flash and boot from Flash\r\n");
    PRINT("    \t  [equivalent to using \"reset fl <firmware path> boot f\" successively]\r\n");

    return CLI_OK;
}

int8_t flb_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    reset_hub(&cli_ref->bhy2);
    if (upload_to_flash((char *)argv[1], &cli_ref->bhy2))
    {
        boot_flash(&cli_ref->bhy2);
        bhy2_get_virt_sensor_list(&cli_ref->bhy2);
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t reset_help(void *ref)
{
    PRINT("  -n OR reset\t= Reset sensor hub\r\n");

    return CLI_OK;
}

int8_t reset_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s\r\n", argv[0]);
    reset_hub(&cli_ref->bhy2);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t addse_help(void *ref)
{
    PRINT("  -a OR addse <sensor id>:<sensor name>:<total output payload in bytes>:\r\n");
    PRINT("     <output_format_0>:<output_format_1>\r\n");
    PRINT("    \t= Register the expected payload of a new custom virtual sensor\r\n");
    PRINT("    \t -Valid output_formats: u8: Unsigned 8 Bit, u16: Unsigned 16 Bit, u32:\r\n");
    PRINT("    \t  Unsigned 32 Bit, s8: Signed 8 Bit, s16: Signed 16 Bit, s32: Signed 32 Bit,\r\n");
    PRINT("    \t  f: Float, c: Char \r\n");
    PRINT("    \t -e.g.: addse 160:\"Lean Orientation\":2:c:c \r\n");
    PRINT("    \t -Note that the corresponding virtual sensor has to be enabled in the same function\r\n");
    PRINT("    \t  call (trailing actse option), since the registration of the sensor is temporary. \r\n");

    return CLI_OK;
}

int8_t addse_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    add_sensor((char *)argv[1], ref);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t rd_help(void *ref)
{
    PRINT("  -r OR rd <adr>[:<len>]\r\n");
    PRINT("    \t= Read from register address <adr> for length <len> bytes\r\n");
    PRINT("    \t -If input <len> is not provided, the default read length is 1 byte\r\n");
    PRINT("    \t -When reading registers with auto-increment, the provided register as well as\r\n");
    PRINT("    \t  the following registers will be read\r\n");
    PRINT("    \t -e.g rd 0x08:3 will read the data of registers 0x08, 0x09 and 0x0a\r\n");
    PRINT("    \t  max. 53 bytes can be read at once\r\n");

    return CLI_OK;
}

int8_t rd_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    rd_regs((char *)argv[1], &cli_ref->bhy2);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t wr_help(void *ref)
{
    PRINT("  -w OR wr <adr>=<val1>[,<val2>]...\r\n");
    PRINT("    \t= Write to register address <adr> with comma separated values <val>\r\n");
    PRINT("    \t -If more values provided <val>, the additional\r\n");
    PRINT("    \t  values will be written to the following addresses\r\n");
    PRINT("    \t -When writing to registers with auto-increment, the provided register as well as\r\n");
    PRINT("    \t  the following registers will be written\r\n");
    PRINT("    \t -e.g wr 0x08=0x02,0x03,0x04 will write the provided data to registers 0x08, 0x09\r\n");
    PRINT("    \t  and 0x0a. Max. 46 bytes can be written at once\r\n");

    return CLI_OK;
}

int8_t wr_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    wr_regs((char *)argv[1], &cli_ref->bhy2);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t rdp_help(void *ref)
{
    PRINT("  -s OR rdp <param id>\r\n");
    PRINT("    \t= Display read_param response of parameter <param id>\r\n");

    return CLI_OK;
}

int8_t rdp_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    rd_param((char *)argv[1], &cli_ref->bhy2);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t wrp_help(void *ref)
{
    PRINT("  -t OR wrp <param id>=<val1>[,<val2>]...\r\n");
    PRINT("    \t= Write data to parameter <param id> with the bytes to be written, <val1>[,<val2>]... \r\n");
    PRINT("    \t -e.g. 0x103=5,6 will write 0x05 to the first byte and 0x06 to the second byte\r\n");
    PRINT("    \t  of the parameter \"Fifo Control\"\r\n");

    return CLI_OK;
}

int8_t wrp_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    wr_param((char *)argv[1], &cli_ref->bhy2);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t physeninfo_help(void *ref)
{
    PRINT("  -p OR physeninfo <physical sensor id>\r\n");
    PRINT("    \t= Display Physical Sensor Information of <physical sensor id>\r\n");

    return CLI_OK;
}

int8_t physeninfo_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    rd_phy_sensor_info((char *)argv[1], &cli_ref->bhy2);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t ram_help(void *ref)
{
    PRINT("  ram <firmware path>\r\n");
    PRINT("    \t= Upload firmware to RAM\r\n");

    return CLI_OK;
}

int8_t ram_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    upload_to_ram((char *)argv[1], &cli_ref->bhy2);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t fl_help(void *ref)
{
    PRINT("  fl <firmware path>\r\n");
    PRINT("    \t= Upload firmware to external-flash\r\n");

    return CLI_OK;
}

int8_t fl_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    upload_to_flash((char *)argv[1], &cli_ref->bhy2);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t boot_help(void *ref)
{
    PRINT("  -g OR boot <medium>\r\n");
    PRINT("    \t= Boot from the specified <medium>: \"f\" for FLASH, \"r\" for RAM\r\n");

    return CLI_OK;
}

int8_t boot_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    if ((argv[1][0]) == 'r')
    {
        boot_ram(&cli_ref->bhy2);
        bhy2_get_virt_sensor_list(&cli_ref->bhy2);
    }
    else if ((argv[1][0]) == 'f')
    {
        boot_flash(&cli_ref->bhy2);
        bhy2_get_virt_sensor_list(&cli_ref->bhy2);
    }
    else
    {
        ERROR("Invalid boot medium: %s\r\n", argv[1]);

        return CLI_E_INVALID_PARAM;
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t erase_help(void *ref)
{
    PRINT("  -e OR erase\t= Erase external-flash\r\n");

    return CLI_OK;
}

int8_t erase_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s \r\n", argv[0]);
    erase_flash(BHY2_FLASH_SIZE_4MB, &cli_ref->bhy2);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t efd_help(void *ref)
{
    PRINT("  efd\t= Erase the flash descriptor\r\n");

    return CLI_OK;
}

int8_t efd_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    struct bhy2_dev *bhy2 = &cli_ref->bhy2;
    int8_t rslt;
    uint8_t boot_status;

    BHY2_ASSERT(bhy2_get_boot_status(&boot_status, bhy2));
    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        if ((boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) || (boot_status & BHY2_BST_FLASH_VERIFY_DONE))
        {
            ERROR("Seems like a firmware is running. Reset the BHI260/BHA260 before erasing the flash descriptor\r\n");

            return CLI_OK;
        }
    }

    PRINT("Erasing flash descriptor. This might hang if a reset command before this was not issued\r\n");
    rslt = bhy2_erase_flash(0, 0xFFF, bhy2); /* 0xFFF is hopefully within the first sector */
    if (rslt != BHY2_OK)
    {
        ERROR("Erasing flash descriptor failed, status: %02d\r\n", rslt);

        return CLI_OK;
    }

    PRINT("Erasing flash descriptor successful\r\n");

    return CLI_OK;
}

int8_t actse_help(void *ref)
{
    PRINT("  -c OR actse <sensor id>:<frequency>[:<latency>]\r\n");
    PRINT("    \t= Activate sensor <sensor id> at specified sample rate <frequency>,\r\n");
    PRINT("    \t -latency <latency>, duration time <time>, sample counts <count>\r\n");
    PRINT("    \t -At least <frequency> is a must input parameter\r\n");
    PRINT("    \t -<latency> is optional\r\n");
    PRINT("    \t -One or more sensors can be active by passing multiple actse options\r\n");
    PRINT("    \t -id: sensor id\r\n");
    PRINT("    \t -frequency(Hz): sensor ODR\r\n");
    PRINT("    \t -latency(ms): sensor data outputs with a latency\r\n");

    return CLI_OK;
}

int8_t actse_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    activate_sensor((char *)argv[1], PARSE_FLAG_STREAM, (struct bhy2_cli_ref *)ref);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t schema_help(void *ref)
{
    PRINT("  schema\t= Show schema information: \r\n");
    PRINT("    \t  Sensor ID, Sensor Name, Event Size, Parsing Format, Axis Names, Scaling Factor\r\n");

    return CLI_OK;
}

int8_t schema_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    schema_info(&cli_ref->bhy2);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t hexse_help(void *ref)
{
    PRINT("  hexse <sensor id>:<frequency>[:<latency>]\r\n");
    PRINT("    \t= Stream sensor <sensor id> at specified sample rate <frequency>, in hex format\r\n");
    PRINT("    \t -latency <latency>, duration time <time>, sample counts <count>\r\n");
    PRINT("    \t -At least <frequency> is a must input parameter\r\n");
    PRINT("    \t -<latency> is optional\r\n");
    PRINT("    \t -One or more sensors can be active by passing multiple logse options\r\n");
    PRINT("    \t -id: sensor id\r\n");
    PRINT("    \t -frequency(Hz): sensor ODR\r\n");
    PRINT("    \t -latency(ms): sensor data outputs with a latency\r\n");

    return CLI_OK;
}

int8_t hexse_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    activate_sensor((char *)argv[1], PARSE_FLAG_HEXSTREAM, (struct bhy2_cli_ref *)ref);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t logse_help(void *ref)
{
    PRINT("  logse <sensor id>:<frequency>[:<latency>]\r\n");
    PRINT("    \t= Log sensor <sensor id> at specified sample rate <frequency>,\r\n");
    PRINT("    \t -latency <latency>, duration time <time>, sample counts <count>\r\n");
    PRINT("    \t -At least <frequency> is a must input parameter\r\n");
    PRINT("    \t -<latency> is optional\r\n");
    PRINT("    \t -One or more sensors can be active by passing multiple logse options\r\n");
    PRINT("    \t -id: sensor id\r\n");
    PRINT("    \t -frequency(Hz): sensor ODR\r\n");
    PRINT("    \t -latency(ms): sensor data outputs with a latency\r\n");

    return CLI_OK;
}

int8_t logse_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    activate_sensor((char *)argv[1], PARSE_FLAG_LOG, (struct bhy2_cli_ref *)ref);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t attlog_help(void *ref)
{
    PRINT("  attlog <filename.ext>\r\n");
    PRINT("    \t= Attach (and create if required) a log file (write-only),");
    PRINT(" where data can be logged to\r\n");

    return CLI_OK;
}

int8_t attlog_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    if (cli_ref->parse_table.logdev.logfile == NULL)
    {
        INFO("Executing %s %s\r\n", argv[0], argv[1]);

        cli_ref->parse_table.logdev.logfile = fopen((char *)argv[1], "wb");
        memcpy(cli_ref->parse_table.logdev.logfilename, (char *)argv[1], strlen((char *)argv[1]));

        if (cli_ref->parse_table.logdev.logfile)
        {
            PRINT("File %s was created\r\n", cli_ref->parse_table.logdev.logfilename);
            write_meta_info(&cli_ref->parse_table.logdev, &cli_ref->bhy2);
        }
        else
        {
            ERROR("File %s could not be found/created\r\n", cli_ref->parse_table.logdev.logfilename);
        }
    }
    else
    {
        ERROR("File %s is open. Please use 'detlog' to detach the open file", cli_ref->parse_table.logdev.logfilename);
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t detlog_help(void *ref)
{
    PRINT("  detlog <filename.ext>\r\n");
    PRINT("    \t= Detach the log file \r\n");

    return CLI_OK;
}

int8_t detlog_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    if (cli_ref->parse_table.logdev.logfile != NULL)
    {
        INFO("Executing %s\r\n", argv[0]);

        if (!strcmp(cli_ref->parse_table.logdev.logfilename, (char *)argv[1]))
        {
            fclose(cli_ref->parse_table.logdev.logfile);
            cli_ref->parse_table.logdev.logfile = NULL;
            PRINT("File %s was detached for logging\r\n", cli_ref->parse_table.logdev.logfilename);
            memset(cli_ref->parse_table.logdev.logfilename, 0, sizeof(cli_ref->parse_table.logdev.logfilename));
        }
        else
        {
            ERROR("Passed Filename does not match with Open File\r\n");
        }
    }
    else
    {
        ERROR("No file to detach\r\n");
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}

int8_t slabel_help(void *ref)
{
    PRINT("  slabel <label string>\r\n");
    PRINT("    \t= Set a string label in the log file %u characters long\r\n", LOGBIN_LABEL_SIZE);

    return CLI_OK;
}

int8_t slabel_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    uint8_t label[LOGBIN_LABEL_SIZE] = { 0 };
    uint64_t timestamp_ns;

    if (cli_ref->parse_table.logdev.logfile != NULL)
    {
        if (strlen((char *)argv[1]) <= LOGBIN_LABEL_SIZE)
        {
            strcpy((char *)label, (char *)argv[1]);
        }
        else
        {
            memcpy(label, argv[1], LOGBIN_LABEL_SIZE);
        }

        INFO("Executing %s %s %s\r\n", argv[0], argv[1], label);
        bhy2_get_hw_timestamp_ns(&timestamp_ns, &cli_ref->bhy2);

        /* System IDs start at 224 */
        log_data(LOGBIN_META_ID_LABEL, timestamp_ns, LOGBIN_LABEL_SIZE, label, &cli_ref->parse_table.logdev);
    }
    else
    {
        ERROR("No open log file to set labels\r\n");
    }

    return CLI_OK;
}

int8_t swim_help(void *ref)
{
    PRINT("  swim <e/d> <r/l> <pool length>\r\n");
    PRINT("    \t<e/d>= Enable / Disable\r\n");
    PRINT("    \t<r/l>= Right / Left\r\n");
    PRINT("    \t<length>= Length of the pool as an integer\r\n");

    return CLI_OK;
}

int8_t swimver_help(void *ref)
{
    PRINT("  swimver\r\n");
    PRINT("    \t= Get the algorithm version\r\n");

    return CLI_OK;
}

int8_t swimgetfreq_help(void *ref)
{
    PRINT("  swimgetfreq\r\n");
    PRINT("    \t To Get the Swim sampling frequency\r\n");

    return CLI_OK;
}

int8_t swimsetfreq_help(void *ref)
{
    PRINT("  swimsetfreq <Freq> <latency>\r\n");
    PRINT("    \t To SET the Swim sampling frequency\r\n");
    PRINT("    \t <Freq> = Frequency (in Hz) to set \r\n");
    PRINT("    \t <Latency> = latency (ms) to set \r\n");

    return CLI_OK;
}

int8_t swimgetaxes_help(void *ref)
{
    PRINT("  swimgetaxes\r\n");
    PRINT("    \t= Get the orientation of Physical sensor set for swim algorithm\r\n");

    return CLI_OK;
}

int8_t swimsetaxes_help(void *ref)
{
    PRINT("  swimsetaxes <orientation_matrix>\r\n");
    PRINT("    \t= Set the orientation of Physical sensor set for swim algorithm\r\n");

    return CLI_OK;
}

int8_t swimgetfreq_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    struct bhy2_virt_sensor_conf sensor_conf;

    INFO("Executing %s\r\n", argv[0]);

    if ((assert_rslt = bhy2_get_virt_sensor_cfg(BHY2_SENSOR_ID_SWIM, &sensor_conf, &cli_ref->bhy2)))
    {
        PRINT("SWIMFREQ GET Failed %d\r\n\r\n\r\n", assert_rslt);
    }
    else
    {
        PRINT("SWIMFREQ %.2f\r\n\r\n\r\n", sensor_conf.sample_rate);
    }

    return CLI_OK;
}

int8_t swimsetfreq_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    bhy2_float sample_rate = 0.0F;
    uint32_t latency = 0;

    INFO("Executing %s\r\n", argv[0]);

    sample_rate = atof((const char *)argv[1]);
    latency = atoi((const char *)argv[2]);

    if ((assert_rslt = bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_SWIM, sample_rate, latency, &cli_ref->bhy2)))
    {
        PRINT("SWIMFREQ SET Failed %d\r\n\r\n\r\n", assert_rslt);
    }
    else
    {
        PRINT("SWIMFREQ SET Success\r\n\r\n\r\n");
    }

    return CLI_OK;
}

int8_t swimver_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    bhy2_swim_version_t swim_algo_ver;
    bhy2_swim_config_param_t config;

    INFO("Executing %s\r\n", argv[0]);

    if (bhy2_swim_get_version(&swim_algo_ver, &cli_ref->bhy2))
    {
        PRINT("SWIMVER 0.0.0\r\n\r\n\r\n");
    }
    else
    {
        if (bhy2_swim_get_config(&config, &cli_ref->bhy2))
        {
            PRINT("SWIMVER %u.%u.%u\r\n\r\n\r\n",
                  swim_algo_ver.improvement,
                  swim_algo_ver.bugfix,
                  swim_algo_ver.platform);
        }
        else
        {
            PRINT("SWIMVER %u.%u.%u\r\nSWIMCONF %s %u\r\n\r\n\r\n",
                  swim_algo_ver.improvement,
                  swim_algo_ver.bugfix,
                  swim_algo_ver.platform,
                  config.dev_on_left_hand ? "LEFT" : "RIGHT",
                  config.pool_length_integral);
        }
    }

    return CLI_OK;
}

int8_t swimgetaxes_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    struct bhy2_orient_matrix orient_matrix;
    uint8_t index;

    INFO("Executing %s\r\n", argv[0]);
    BHY2_ASSERT(bhy2_get_orientation_matrix(BHY2_PHYS_SENSOR_ID_ACCELEROMETER, &orient_matrix, &cli_ref->bhy2));
    PRINT("Acc ");

    for (index = 0; index < 8; index++)
    {
        PRINT("%d,", orient_matrix.c[index]);
    }

    PRINT("%d\r\n\r\n", orient_matrix.c[index]);

    memset(&orient_matrix.c[0], 0x0, sizeof(orient_matrix.c) / sizeof(orient_matrix.c[0]));
    BHY2_ASSERT(bhy2_get_orientation_matrix(BHY2_PHYS_SENSOR_ID_GYROSCOPE, &orient_matrix, &cli_ref->bhy2));
    PRINT("Gyro ");

    for (index = 0; index < 8; index++)
    {
        PRINT("%d,", orient_matrix.c[index]);
    }

    PRINT("%d\r\n\r\n", orient_matrix.c[index]);

    return CLI_OK;
}

int8_t swimsetaxes_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    struct bhy2_orient_matrix orient_matrix;
    uint8_t index = 0;
    char delimiter[] = ",";
    char *axes = (char *)argv[1];
    char *token = strtok(axes, delimiter);

    while ((token != NULL))
    {
        orient_matrix.c[index] = atoi(token);
        token = strtok(NULL, delimiter);
        index++;
    }

    INFO("Executing %s %s\r\n", argv[0], argv[1]);

    BHY2_ASSERT(bhy2_set_orientation_matrix(BHY2_PHYS_SENSOR_ID_ACCELEROMETER, orient_matrix, &cli_ref->bhy2));
    BHY2_ASSERT(bhy2_set_orientation_matrix(BHY2_PHYS_SENSOR_ID_GYROSCOPE, orient_matrix, &cli_ref->bhy2));

    PRINT("Set the orientation matrix for the Physical Sensors successfully");

    return CLI_OK;
}

int8_t swim_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    bhy2_swim_config_param_t config;

    INFO("Executing %s %s %s %s\r\n", argv[0], argv[1], argv[2], argv[3]);

    config.update_swim_config = (argv[1][0] == 'e') ? BHY2_SWIM_ENABLE_CONFIG : BHY2_SWIM_DISABLE_CONFIG;
    config.dev_on_left_hand = (argv[2][0] == 'r') ? BHY2_SWIM_DEVICE_ON_RIGHT_HAND : BHY2_SWIM_DEVICE_ON_LEFT_HAND;
    config.pool_length_integral = atoi((char *)argv[3]);
    config.pool_length_floating = 0;

    BHY2_ASSERT(bhy2_swim_set_config(&config, &cli_ref->bhy2));

    PRINT("Setting swim config: %s, %s, %u\r\n\r\n",
          (argv[1][0] == 'e') ? "Enable" : "Disable",
          (argv[2][0] == 'r') ? "Right" : "Left",
          atoi((char *)argv[3]));

    /*! Writing the final swim output after disabling swim sensor*/
    if (config.update_swim_config == BHY2_SWIM_DISABLE_CONFIG)
    {
        PRINT("Summary D: %u; C: %u; FRS: %u; BRS: %u; BTS: %u; BKS: %u; STC: %u\r\n",
              swim_data.total_distance,
              swim_data.length_count,
              swim_data.lengths_freestyle,
              swim_data.lengths_breaststroke,
              swim_data.lengths_butterfly,
              swim_data.lengths_backstroke,
              swim_data.stroke_count);
    }

    return CLI_OK;
}

static void time_to_s_ns(uint64_t time_ticks, uint32_t *s, uint32_t *ns, uint64_t *tns)
{
    *tns = time_ticks * 15625; /* timestamp is now in nanoseconds */
    *s = (uint32_t)(*tns / UINT64_C(1000000000));
    *ns = (uint32_t)(*tns - ((*s) * UINT64_C(1000000000)));
}

void parse_klio(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    bhy2_klio_sensor_frame_t data;
    uint8_t parse_flag;
    uint32_t klio_driver_status;
    uint8_t tmp_buf[252];
    uint16_t bufsize = sizeof(tmp_buf);
    bhy2_klio_sensor_state_t klio_sensor_state;
    struct bhy2_dev *bhy2;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    bhy2 = parse_table->bhy2;

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    memcpy(&data, callback_info->data_ptr, sizeof(data));

    if (data.learn.index != -1)
    {
        /* Read out learnt pattern */
        BHY2_ASSERT(bhy2_klio_read_pattern(0, tmp_buf, &bufsize, bhy2));
        BHY2_ASSERT(bhy2_klio_read_reset_driver_status(&klio_driver_status, bhy2));

        DATA("SID: %u; T: %lu.%09lu; Pattern learnt: ", callback_info->sensor_id, s, ns);
        for (uint16_t i = 0; i < bufsize; i++)
        {
            PRINT_D("%02x", tmp_buf[i]);
        }

        PRINT_D("\r\n");

        /* write back learnt pattern for recognition */
        if (klio_vars.auto_load_pattern && klio_vars.auto_load_pattern_write_index < klio_vars.max_patterns)
        {
            BHY2_ASSERT(bhy2_klio_write_pattern(klio_vars.auto_load_pattern_write_index, tmp_buf, bufsize, bhy2));
            BHY2_ASSERT(bhy2_klio_read_reset_driver_status(&klio_driver_status, bhy2));

            BHY2_ASSERT(bhy2_klio_set_pattern_states(KLIO_PATTERN_STATE_ENABLE,
                                                     &klio_vars.auto_load_pattern_write_index, 1, bhy2));
            BHY2_ASSERT(bhy2_klio_read_reset_driver_status(&klio_driver_status, bhy2));

            klio_vars.auto_load_pattern_write_index++;

            /* write klio state (enable recognition, and also make sure learning is not disabled) */
            klio_sensor_state.learning_enabled = 1;
            klio_sensor_state.learning_reset = 0;
            klio_sensor_state.recognition_enabled = 1;
            klio_sensor_state.recognition_reset = 0;
            BHY2_ASSERT(bhy2_klio_set_state(&klio_sensor_state, bhy2));
            BHY2_ASSERT(bhy2_klio_read_reset_driver_status(&klio_driver_status, bhy2));
        }
    }

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; Learning [Id:%d Progress:%u Change:%u]; Recognition[Id:%d Count:%f]\r\n",
             callback_info->sensor_id,
             s,
             ns,
             data.learn.index,
             data.learn.progress,
             data.learn.change_reason,
             data.recognize.index,
             data.recognize.count);
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

void parse_klio_log(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    bhy2_klio_log_frame_t data;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    memcpy(&data, callback_info->data_ptr, sizeof(data));

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; ax: %.9g, ay: %.9g, az: %.9g, gx: %.9g, gy: %.9g, gz: %.9g\r\n",
             callback_info->sensor_id,
             s,
             ns,
             data.accel[0],
             data.accel[1],
             data.accel[2],
             data.gyro[0],
             data.gyro[1],
             data.gyro[2]);
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

void parse_acc_gyro(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint8_t parse_flag = 0;
    uint32_t s = 0, ns = 0;
    uint64_t tns = 0;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    float *sensor_data = (float *)callback_info->data_ptr;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;
    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; X: %f; Y: %f; Z: %f;\r\n",
             callback_info->sensor_id,
             s,
             ns,
             sensor_data[0],
             sensor_data[1],
             sensor_data[2]);
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

void parse_air_quality(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint8_t parse_flag = 0;
    uint32_t s = 0, ns = 0;
    uint64_t tns = 0;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    struct parse_sensor_details *sensor_details;
    struct bhy2_bsec_air_quality aq = { 0 };

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;
    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    bhy2_bsec_parse_air_quality(callback_info->data_ptr, &aq);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; T: %.2f, H: %.2f, G: %.2f, I: %.2f, S: %.2f, C: %.2f, V: %.2f, A: %u\r\n",
             callback_info->sensor_id,
             s,
             ns,
             aq.comp_temp,
             aq.comp_hum,
             aq.comp_gas,
             aq.iaq,
             aq.static_iaq,
             aq.e_co2,
             aq.voc,
             aq.iaq_accuracy);
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

void parse_swim(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;
    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    bhy2_swim_parse_data(callback_info->data_ptr, &swim_data);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; D: %u; C: %u; FRS: %u; BRS: %u; BTF: %u; BKS: %u; STC: %u\r\n",
             callback_info->sensor_id,
             s,
             ns,
             swim_data.total_distance,
             swim_data.length_count,
             swim_data.lengths_freestyle,
             swim_data.lengths_breaststroke,
             swim_data.lengths_butterfly,
             swim_data.lengths_backstroke,
             swim_data.stroke_count);
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

void parse_multitap(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    enum bhi3_multi_tap_val multitap_data;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;
    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    bhi3_multi_tap_parse_data(callback_info->data_ptr, (uint8_t *)&multitap_data);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; %s; \r\n",
             callback_info->sensor_id,
             s,
             ns,
             bhi3_multi_tap_string_out[multitap_data]);
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

void parse_wrist_gesture_detect(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    struct bhi3_wrist_gesture_detect wrist_gesture_detect_data = { 0 };

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;
    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    bhi3_wrist_gesture_detect_parse_data(callback_info->data_ptr, &wrist_gesture_detect_data);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; wrist_gesture: %s; \r\n",
             callback_info->sensor_id,
             s,
             ns,
             bhi3_wrist_gesture_detect_output[wrist_gesture_detect_data.wrist_gesture]);
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

void parse_hmc(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhy2_head_tracker_quat_data data;
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

    bhy2_head_tracker_quat_parsing(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f, w: %f; acc: %u\r\n",
             callback_info->sensor_id,
             s,
             ns,
             data.x / 16384.0f,
             data.y / 16384.0f,
             data.z / 16384.0f,
             data.w / 16384.0f,
             data.accuracy);
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

void parse_oc(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhy2_head_tracker_quat_data data;
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

    bhy2_head_tracker_quat_parsing(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f, w: %f; acc: %u\r\n",
             callback_info->sensor_id,
             s,
             ns,
             data.x / 16384.0f,
             data.y / 16384.0f,
             data.z / 16384.0f,
             data.w / 16384.0f,
             data.accuracy);
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

void parse_ec(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhy2_head_tracker_eul_data data;
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

    bhy2_head_tracker_eul_parsing(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; h: %f, p: %f, r: %f; acc: %u\r\n",
             callback_info->sensor_id,
             s,
             ns,
             (data.heading * 360.0f) / 32768.0f,
             (data.pitch * 360.0f) / 32768.0f,
             (data.roll * 360.0f) / 32768.0f,
             data.accuracy);
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

void bhy2_install_callbacks(struct bhy2_dev *bhy2, struct parse_ref *parse_table)
{
    for (uint8_t i = 0; i < BHY2_MAX_SIMUL_SENSORS; i++)
    {
        parse_table->sensor[i].parse_flag = PARSE_FLAG_NONE;
        parse_table->sensor[i].id = 0;
    }

    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, parse_table, bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event, parse_table, bhy2));
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(BHY2_SYS_ID_DEBUG_MSG, parse_debug_message, parse_table, bhy2));
}

static void check_bhy2_api(unsigned int line, const char *func, int8_t val)
{
    ERROR("BHI260 API failed at line %u. The function %s returned error code %d. %s\r\n",
          line,
          func,
          val,
          get_api_error(val));
}

static void reset_hub(struct bhy2_dev *bhy2)
{
    uint8_t data = 0, data_exp;

    BHY2_ASSERT(bhy2_soft_reset(bhy2));

    BHY2_ASSERT(bhy2_get_host_interrupt_ctrl(&data, bhy2));
    data &= ~BHY2_ICTL_DISABLE_STATUS_FIFO; /* Enable status interrupts */
    data &= ~BHY2_ICTL_DISABLE_DEBUG; /* Enable debug interrupts */
    data &= ~BHY2_ICTL_EDGE; /* Level */
    data &= ~BHY2_ICTL_ACTIVE_LOW; /* Active high */
    data &= ~BHY2_ICTL_OPEN_DRAIN; /* Push-pull */
    data_exp = data;
    BHY2_ASSERT(bhy2_set_host_interrupt_ctrl(data, bhy2));
    BHY2_ASSERT(bhy2_get_host_interrupt_ctrl(&data, bhy2));
    if (data != data_exp)
    {
        WARNING("Expected Host Interrupt Control (0x07) to have value 0x%x but instead read 0x%x\r\n", data_exp, data);
    }

    /* Config status channel */
    BHY2_ASSERT(bhy2_set_host_intf_ctrl(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL, bhy2));
    BHY2_ASSERT(bhy2_get_host_intf_ctrl(&data, bhy2));
    if (!(data & BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL))
    {
        WARNING("Expected Host Interface Control (0x06) to have bit 0x%x to be set\r\n",
                BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL);
    }

    PRINT("Reset successful\r\n");
}

static bool upload_to_ram(const char *filepath, struct bhy2_dev *bhy2)
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
    BHY2_ASSERT(bhy2_get_boot_status(&boot_status, bhy2));
    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        if ((boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) || (boot_status & BHY2_BST_FLASH_VERIFY_DONE))
        {
            ERROR("Seems like a firmware is running. Please reset the BHI260/BHA260 before uploading firmware\r\n");

            return false;
        }

#ifdef PC
        fw_file = fopen(filepath, "rb"); /* Without the b, the file is read incorrectly */
#else
        fw_file = fopen(filepath, "r");
#endif

        if (!fw_file)
        {
            ERROR("Cannot open file: %s\r\n", filepath);

            return false;
        }

        stat(filepath, &st);
        len = st.st_size;

        /* 256 KB */
        if (len > 262144)
        {
            ERROR("Invalid RAM Size of %lu bytes\r\n", len);

            return false;
        }

        PRINT("Uploading %lu bytes of firmware to RAM\r\n", len);
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
            rslt = bhy2_upload_firmware_to_ram_partly(firmware_chunk, len, i, incr, bhy2);
#ifdef PC
            progress = (float)(i + incr) / (float)len * 100.0f;
            if (progress != new_progress)
            {
                INFO("Completed %u %%\r", progress);
                new_progress = progress;
            }

#endif
        }

        INFO("Firmware upload took %.2f seconds\r\n", (float)(coines_get_millis() - start_time_ms) / 1000.0f);
        fclose(fw_file);

        if (rslt != BHY2_OK)
        {
            ERROR("Firmware upload failed. Returned with error code: %d. %s\r\n", rslt, get_api_error(rslt));

            return false;
        }
    }
    else
    {
        ERROR("Host interface is not ready\r\n");

        return false;
    }

    PRINT("Uploading firmware to RAM successful\r\n");

    return true;
}

static void print_boot_status(uint8_t boot_status)
{
    PRINT("Boot Status : 0x%02x: ", boot_status);
    if (boot_status & BHY2_BST_FLASH_DETECTED)
    {
        PRINT("Flash detected. ");
    }

    if (boot_status & BHY2_BST_FLASH_VERIFY_DONE)
    {
        PRINT("Flash verify done. ");
    }

    if (boot_status & BHY2_BST_FLASH_VERIFY_ERROR)
    {
        PRINT("Flash verification failed. ");
    }

    if (boot_status & BHY2_BST_NO_FLASH)
    {
        PRINT("No flash installed. ");
    }

    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        PRINT("Host interface ready. ");
    }

    if (boot_status & BHY2_BST_HOST_FW_VERIFY_DONE)
    {
        PRINT("Firmware verification done. ");
    }

    if (boot_status & BHY2_BST_HOST_FW_VERIFY_ERROR)
    {
        PRINT("Firmware verification error. ");
    }

    if (boot_status & BHY2_BST_HOST_FW_IDLE)
    {
        PRINT("Firmware halted. ");
    }

    PRINT("\r\n");
}

static void boot_ram(struct bhy2_dev *bhy2)
{
    int8_t rslt;
    uint8_t feat_status;
    uint8_t boot_status, error_val;
    uint16_t tries = 100;

    PRINT("Waiting for firmware verification to complete\r\n");
    do
    {
        bhy2->hif.delay_us(10000, NULL);
        BHY2_ASSERT(bhy2_get_boot_status(&boot_status, bhy2));
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
            INFO("Booting from RAM\r\n");
            rslt = bhy2_boot_from_ram(bhy2);
            if (rslt != BHY2_OK)
            {
                ERROR("Booting from RAM failed. API error code %d.\r\n%s\r\n", rslt, get_api_error(rslt));
                BHY2_ASSERT(bhy2_get_error_value(&error_val, bhy2));
                if (error_val)
                {
                    ERROR("Sensor reports error 0x%X.\r\n%s", error_val, get_sensor_error_text(error_val));
                }

                return;
            }

            BHY2_ASSERT(bhy2_get_feature_status(&feat_status, bhy2));
            if (feat_status & BHY2_FEAT_STATUS_OPEN_RTOS_MSK)
            {
                BHY2_ASSERT(bhy2_update_virtual_sensor_list(bhy2));

                for (uint16_t i = 0; i < 10; i++)
                {
                    /* Process meta events over a period of 100ms*/
                    BHY2_ASSERT(bhy2_get_and_process_fifo(fifo_buffer, sizeof(fifo_buffer), bhy2));
                    bhy2->hif.delay_us(10000, NULL);
                }
            }
            else
            {
                ERROR("Reading Feature status failed, booting from RAM failed\r\n");

                return;
            }
        }
        else
        {
            ERROR("Upload firmware to RAM before boot\r\n");

            return;
        }
    }
    else
    {
        ERROR("Host interface is not ready\r\n");

        return;
    }

    PRINT("Booting from RAM successful\r\n");
}

void show_info(struct bhy2_dev *bhy2)
{
    uint16_t kernel_version = 0, user_version = 0;
    uint16_t rom_version = 0;
    uint8_t product_id = 0;
    uint8_t host_status = 0, feat_status = 0;
    uint8_t val = 0;
    uint8_t sensor_error;
    struct bhy2_sensor_info info;

    /* Get product_id */
    BHY2_ASSERT(bhy2_get_product_id(&product_id, bhy2));

    /* Get Kernel version */
    BHY2_ASSERT(bhy2_get_kernel_version(&kernel_version, bhy2));

    /* Get User version */
    BHY2_ASSERT(bhy2_get_user_version(&user_version, bhy2));

    /* Get ROM version */
    BHY2_ASSERT(bhy2_get_rom_version(&rom_version, bhy2));

    BHY2_ASSERT(bhy2_get_host_status(&host_status, bhy2));

    BHY2_ASSERT(bhy2_get_feature_status(&feat_status, bhy2));

    PRINT("Product ID     : %02x\r\n", product_id);
    PRINT("Kernel version : %04u\r\n", kernel_version);
    PRINT("User version   : %04u\r\n", user_version);
    PRINT("ROM version    : %04u\r\n", rom_version);
    PRINT("Power state    : %s\r\n", (host_status & BHY2_HST_POWER_STATE) ? "sleeping" : "active");
    PRINT("Host interface : %s\r\n", (host_status & BHY2_HST_HOST_PROTOCOL) ? "SPI" : "I2C");
    PRINT("Feature status : 0x%02x\r\n", feat_status);

    /* Read boot status */
    BHY2_ASSERT(bhy2_get_boot_status(&val, bhy2));
    print_boot_status(val);

    /* Read error value */
    BHY2_ASSERT(bhy2_get_error_value(&sensor_error, bhy2));
    if (sensor_error)
    {
        ERROR("%s\r\n", get_sensor_error_text(sensor_error));
    }

    if (feat_status & BHY2_FEAT_STATUS_OPEN_RTOS_MSK)
    {
        bhy2_update_virtual_sensor_list(bhy2);

        /* Get present virtual sensor */
        bhy2_get_virt_sensor_list(bhy2);

        PRINT("Virtual sensor list.\r\n");
        PRINT("Sensor ID |                          Sensor Name |  ID | Ver |  Min rate |  Max rate |\r\n");
        PRINT("----------+--------------------------------------+-----+-----+-----------+-----------|\r\n");
        for (uint8_t i = 0; i < BHY2_SENSOR_ID_MAX; i++)
        {
            if (bhy2_is_sensor_available(i, bhy2))
            {
                if (i < BHY2_SENSOR_ID_CUSTOM_START)
                {
                    PRINT(" %8u | %36s ", i, get_sensor_name(i));
                }
                else
                {
                    PRINT(" %8u | %36s ", i, custom_driver_information[i - BHY2_SENSOR_ID_CUSTOM_START].sensor_name);
                }

                BHY2_ASSERT(bhy2_get_sensor_info(i, &info, bhy2));
                PRINT("| %3u | %3u | %9.4f | %9.4f |\r\n",
                      info.driver_id,
                      info.driver_version,
                      info.min_rate.f_val,
                      info.max_rate.f_val);
            }
        }
    }
}

static void boot_flash(struct bhy2_dev *bhy2)
{
    int8_t rslt;
    uint8_t boot_status, feat_status;
    uint8_t error_val = 0;
    uint16_t tries = 300; /* Wait for up to little over 3s */

    PRINT("Waiting for firmware verification to complete\r\n");
    do
    {
        bhy2->hif.delay_us(10000, NULL);
        BHY2_ASSERT(bhy2_get_boot_status(&boot_status, bhy2));
        if (boot_status & BHY2_BST_FLASH_VERIFY_DONE)
        {
            break;
        }
    } while (tries--);

    BHY2_ASSERT(bhy2_get_boot_status(&boot_status, bhy2));
    print_boot_status(boot_status);
    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        if (boot_status & BHY2_BST_FLASH_DETECTED)
        {

            /* If no firmware is running, boot from Flash */
            PRINT("Booting from flash\r\n");
            rslt = bhy2_boot_from_flash(bhy2);
            if (rslt != BHY2_OK)
            {
                ERROR("%s. Booting from flash failed.\r\n", get_api_error(rslt));
                BHY2_ASSERT(bhy2_get_regs(BHY2_REG_ERROR_VALUE, &error_val, 1, bhy2));
                if (error_val)
                {
                    ERROR("%s\r\n", get_sensor_error_text(error_val));
                }

                return;
            }

            BHY2_ASSERT(bhy2_get_boot_status(&boot_status, bhy2));
            print_boot_status(boot_status);

            if (!(boot_status & BHY2_BST_HOST_INTERFACE_READY))
            {
                /* hub is not ready, need reset hub */
                PRINT("Host interface is not ready, triggering a reset\r\n");

                BHY2_ASSERT(bhy2_soft_reset(bhy2));
            }

            BHY2_ASSERT(bhy2_get_feature_status(&feat_status, bhy2));
            if (feat_status & BHY2_FEAT_STATUS_OPEN_RTOS_MSK)
            {
                BHY2_ASSERT(bhy2_update_virtual_sensor_list(bhy2));
                for (uint16_t i = 0; i < 10; i++) /* Process meta events */
                {
                    BHY2_ASSERT(bhy2_get_and_process_fifo(fifo_buffer, sizeof(fifo_buffer), bhy2));
                    bhy2->hif.delay_us(10000, NULL);
                }
            }
            else
            {
                ERROR("Reading Feature status failed, booting from flash failed\r\n");

                return;
            }
        }
        else
        {
            ERROR("Can't detect external flash\r\n");

            return;
        }
    }
    else
    {
        ERROR("Host interface is not ready\r\n");

        return;
    }

    PRINT("Booting from flash successful\r\n");
}

static bool upload_to_flash(const char *filepath, struct bhy2_dev *bhy2)
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
    BHY2_ASSERT(bhy2_get_boot_status(&boot_status, bhy2));
    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        if ((boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) || (boot_status & BHY2_BST_FLASH_VERIFY_DONE))
        {
            ERROR("Reset the BHI260/BHA260 before uploading firmware to external flash\r\n");

            return false;
        }

#ifdef PC
        fw_file = fopen(filepath, "rb"); /* Without the b, the file is read incorrectly */
#else
        fw_file = fopen(filepath, "r");
#endif
        if (!fw_file)
        {
            ERROR("Cannot open file: %s\r\n", filepath);

            return false;
        }

        stat(filepath, &st);
        len = st.st_size;

        /* 8 MB */
        if (len > 8388608)
        {
            ERROR("Invalid firmware size of %lu bytes\r\n", len);

            return false;
        }

        PRINT("Erasing first %lu bytes of flash\r\n", len);
        rslt = bhy2_erase_flash(BHY2_FLASH_SECTOR_START_ADDR, BHY2_FLASH_SECTOR_START_ADDR + len, bhy2);
        if (rslt != BHY2_OK)
        {
            ERROR("Erasing flash failed with error code %d. %s\r\n", rslt, get_api_error(rslt));
        }

        PRINT("Uploading %lu bytes of firmware to flash\r\n", len);
        start_time_ms = coines_get_millis();
        uint32_t incr = BHY2_RD_WR_LEN;
        if ((incr % 4) != 0) /* Round of to lower 4 bytes */
        {
            incr = BHY2_ROUND_WORD_LOWER(incr);
        }

        for (uint32_t i = 0; (i < len) && (rslt == BHY2_OK); i += incr)
        {
            if (incr > (len - i)) /* Last payload */
            {
                incr = len - i;
                if ((incr % 4) != 0) /* Round of to higher 4bytes */
                {
                    incr = BHY2_ROUND_WORD_HIGHER(incr);
                }
            }

            fread(firmware_chunk, 1, incr, fw_file);
            rslt = bhy2_upload_firmware_to_flash_partly(firmware_chunk, i, incr, bhy2);
#ifdef PC
            progress = (float)(i + incr) / (float)len * 100.0f;
            if (progress != new_progress)
            {
                INFO("Completed %u %%\r", progress);
                new_progress = progress;
            }

#endif
        }

        INFO("Firmware upload took %.2f seconds\r\n", (float)(coines_get_millis() - start_time_ms) / 1000.0f);
        fclose(fw_file);
        if (rslt != BHY2_OK)
        {
            ERROR("%s. Firmware upload failed\r\n", get_api_error(rslt));

            return false;
        }
    }
    else
    {
        ERROR("Host interface is not ready\r\n");
    }

    PRINT("Uploading firmware to flash successful\r\n");

    return true;
}

static void wr_regs(const char *payload, struct bhy2_dev *bhy2)
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
    start = (char *)payload;
    end = strchr(start, '=');
    if (end == NULL)
    {
        ERROR("Write address format error\r\n");

        return;
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
    BHY2_ASSERT(bhy2_set_regs(reg, val, len, bhy2));

    PRINT("Writing address successful\r\n");
}

static void rd_regs(const char *payload, struct bhy2_dev *bhy2)
{
    char *start;
    char *end;
    char str_reg[8] = { 0 };
    uint8_t reg;
    uint8_t val[1024];
    uint16_t len;
    uint16_t i = 0;
    uint16_t j = 0;

    start = (char *)payload;
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
    BHY2_ASSERT(bhy2_get_regs(reg, val, len, bhy2));

    /* Print register data to console */

    /* Registers after the status channel are auto increment,
     * reading more than 1 byte will lead to reading
     * the subsequent register addresses */
    if (reg <= BHY2_REG_CHAN_STATUS)
    {
        PRINT("Reading from register address 0x%02x:\r\n", reg);
        PRINT("Byte hex       dec | Data\r\n");
        PRINT("-------------------------------------------\r\n");
        for (i = 0; i < len; i++)
        {
            if (j == 0)
            {
                PRINT(" ");
                PRINT("0x%06x %8d |", i, i);
            }

            PRINT(" %02x", val[i]);
            ++j;
            if (j >= 8)
            {
                PRINT("\r\n");
                j = 0;
            }
        }

        if ((len % 8) == 0)
        {
            PRINT("\r\n");
        }
    }
    else
    {
        PRINT("Register address: Data\r\n");
        PRINT("----------------------\r\n");
        for (i = 0; i < len; i++)
        {
            PRINT("0x%02x            : %02x \r\n", reg + i, val[i]);
        }
    }

    PRINT("Read complete\r\n");
}

void rd_param(const char *payload, struct bhy2_dev *bhy2)
{
    char str_param_id[8] = { 0 };
    uint8_t tmp_buf[1024] = { 0 };
    uint16_t param_id;
    uint32_t ret_len = 0;
    uint16_t i;
    uint16_t j = 0;

    strncpy(str_param_id, payload, strlen(payload));
    str_param_id[strlen(payload)] = '\0';
    param_id = (uint16_t)strtol(str_param_id, NULL, 0);

    BHY2_ASSERT(bhy2_get_parameter(param_id, tmp_buf, sizeof(tmp_buf), &ret_len, bhy2));
    if (assert_rslt != BHY2_OK)
    {
        return;
    }

    PRINT("Byte hex      dec | Data\r\n");
    PRINT("-------------------------------------------\r\n");
    for (i = 0; i < ret_len; i++)
    {
        if (j == 0)
        {
            PRINT("0x%06x %8d |", i, i);
        }

        PRINT("%02x ", tmp_buf[i]);
        j++;
        if (j >= 8)
        {
            PRINT("\r\n");
            j = 0;
        }
    }

    if ((ret_len % 8) != 0)
    {
        PRINT("\r\n");
    }

    PRINT("Reading parameter 0x%04X successful\r\n", param_id);
}

static void wr_param(const char *payload, struct bhy2_dev *bhy2)
{
    char *start, *end;
    char str_param_id[8] = { 0 };
    char str_data[8] = { 0 };
    uint8_t data_buf[1024] = { 0 };
    uint16_t param_id;
    uint8_t val;
    uint16_t buf_size = 0;
    uint8_t break_out = 0;

    start = (char *)payload;
    end = strchr(start, '=');
    if (end == NULL)
    {
        ERROR("Write parameter I/O format error\r\n");

        return;
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

        strncpy(str_data, start, (size_t)(end - start));
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

    BHY2_ASSERT(bhy2_set_parameter(param_id, data_buf, buf_size, bhy2));
    PRINT("Writing parameter successful\r\n");
}

void rd_phy_sensor_info(const char *payload, struct bhy2_dev *bhy2)
{
    uint16_t param_id;
    uint8_t sens_id;
    struct bhy2_phys_sensor_info psi = { 0 };

    sens_id = (uint8_t)atoi((char *)&payload[0]);
    param_id = (uint16_t)(0x0120 | sens_id);

    if (param_id >= 0x0121 && param_id <= 0x0160)
    {
        BHY2_ASSERT(bhy2_get_phys_sensor_info(sens_id, &psi, bhy2));
        if (assert_rslt != BHY2_OK)
        {
            return;
        }

        PRINT("Field Name            hex                    | Value (dec)\r\n");
        PRINT("----------------------------------------------------------\r\n");
        PRINT("Physical Sensor ID    %02X                     | %d\r\n", psi.sensor_type, psi.sensor_type);
        PRINT("Driver ID             %02X                     | %d\r\n", psi.driver_id, psi.driver_id);
        PRINT("Driver Version        %02X                     | %d\r\n", psi.driver_version, psi.driver_version);
        PRINT("Current Consumption   %02X                     | %0.3fmA\r\n",
              psi.power_current,
              psi.power_current / 10.f);
        PRINT("Dynamic Range         %04X                   | %d\r\n", psi.curr_range.u16_val, psi.curr_range.u16_val);

        const char *irq_status[2] = { "Disabled", "Enabled" };
        const char *master_intf[5] = { "None", "SPI0", "I2C0", "SPI1", "I2C1" };
        const char *power_mode[8] =
        { "Sensor Not Present", "Power Down", "Suspend", "Self-Test", "Interrupt Motion", "One Shot",
          "Low Power Active", "Active" };

        PRINT("Flags                 %02X                     | IRQ status       : %s\r\n", psi.flags,
              irq_status[psi.flags & 0x01]);
        PRINT("                                             | Master interface : %s\r\n",
              master_intf[(psi.flags >> 1) & 0x0F]);
        PRINT("                                             | Power mode       : %s\r\n",
              power_mode[(psi.flags >> 5) & 0x07]);
        PRINT("Slave Address         %02X                     | %d\r\n", psi.slave_address, psi.slave_address);
        PRINT("GPIO Assignment       %02X                     | %d\r\n", psi.gpio_assignment, psi.gpio_assignment);
        PRINT("Current Rate          %08X               | %.3fHz\r\n", psi.curr_rate.u32_val, psi.curr_rate.f_val);
        PRINT("Number of axes        %02X                     | %d\r\n", psi.num_axis, psi.num_axis);

        #define INT4_TO_INT8(INT4)  ((int8_t)(((INT4) > 1) ? -1 : (INT4)))
        struct bhy2_orient_matrix ort_mtx = { 0 };
        ort_mtx.c[0] = INT4_TO_INT8(psi.orientation_matrix[0] & 0x0F);
        ort_mtx.c[1] = INT4_TO_INT8(psi.orientation_matrix[0] >> 8);
        ort_mtx.c[2] = INT4_TO_INT8(psi.orientation_matrix[1] & 0x0F);
        ort_mtx.c[3] = INT4_TO_INT8(psi.orientation_matrix[1] >> 8);
        ort_mtx.c[4] = INT4_TO_INT8(psi.orientation_matrix[2] & 0x0F);
        ort_mtx.c[5] = INT4_TO_INT8(psi.orientation_matrix[2] >> 8);
        ort_mtx.c[6] = INT4_TO_INT8(psi.orientation_matrix[3] & 0x0F);
        ort_mtx.c[7] = INT4_TO_INT8(psi.orientation_matrix[3] >> 8);
        ort_mtx.c[8] = INT4_TO_INT8(psi.orientation_matrix[4] & 0x0F);

        PRINT("Orientation Matrix    %02X%02X%02X%02X%02X             | %+02d %+02d %+02d |\r\n",
              psi.orientation_matrix[0],
              psi.orientation_matrix[1],
              psi.orientation_matrix[2],
              psi.orientation_matrix[3],
              psi.orientation_matrix[4],
              ort_mtx.c[0],
              ort_mtx.c[1],
              ort_mtx.c[2]);
        PRINT("                                             | %+02d %+02d %+02d |\r\n",
              ort_mtx.c[3],
              ort_mtx.c[4],
              ort_mtx.c[5]);
        PRINT("                                             | %+02d %+02d %+02d |\r\n",
              ort_mtx.c[6],
              ort_mtx.c[7],
              ort_mtx.c[8]);
        PRINT("Reserved              %02X                     | %d\r\n", psi.reserved, psi.reserved);
        PRINT("\r\n");

    }
}

static void erase_flash(uint32_t end_addr, struct bhy2_dev *bhy2)
{
    int8_t rslt;
    uint8_t boot_status;

    BHY2_ASSERT(bhy2_get_boot_status(&boot_status, bhy2));
    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        if ((boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) || (boot_status & BHY2_BST_FLASH_VERIFY_DONE))
        {
            ERROR("Seems like a firmware is running. Reset the BHI260/BHA260 before erasing external flash\r\n");

            return;
        }
    }

    PRINT("Erasing flash. May take a while\r\n");
    rslt = bhy2_erase_flash(BHY2_FLASH_SECTOR_START_ADDR, BHY2_FLASH_SECTOR_START_ADDR + end_addr, bhy2);
    if (rslt != BHY2_OK)
    {
        ERROR("Erasing flash failed, status: %02d\r\n", rslt);

        return;
    }

    PRINT("Erasing flash successful\r\n");
}

static void activate_sensor(const char *sensor_parameters, uint8_t parse_flag, struct bhy2_cli_ref *ref)
{
    struct bhy2_dev *bhy2 = &(ref->bhy2);
    struct parse_ref *parse_table = &(ref->parse_table);
    bhy2_fifo_parse_callback_t callback;
    struct parse_sensor_details *sensor_details;

    char sen_id_str[8], sample_rate_str[8], sen_latency_str[8];
    uint8_t sen_id;
    uint32_t sen_latency = 0;
    float sample_rate;
    char *start, *end;

    /* Parse Sensor ID */
    start = (char *)sensor_parameters;
    end = strchr(start, ':');
    if (end == NULL)
    {
        ERROR("Sensor ID / Sample rate format error\r\n");

        return;
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

    if (first_run)
    {
        first_run = false;
        bhy2_update_virtual_sensor_list(bhy2);

        /* Get present virtual sensor */
        bhy2_get_virt_sensor_list(bhy2);
    }

    /* If the payload of this sensor is not yet registered and within the custom virtual sensor id range, register the
     * default parsing function */
    if (bhy2_is_sensor_available(sen_id, bhy2))
    {
        if ((sen_id >= BHY2_SENSOR_ID_CUSTOM_START) && (sen_id <= BHY2_SENSOR_ID_CUSTOM_END) &&
            (custom_driver_information[sen_id - BHY2_SENSOR_ID_CUSTOM_START].is_registered != 1))
        {
            custom_driver_information[sen_id - BHY2_SENSOR_ID_CUSTOM_START].sensor_payload = bhy2->event_size[sen_id];

            BHY2_ASSERT(bhy2_register_fifo_parse_callback(sen_id, parse_custom_sensor_default, parse_table, bhy2));
            PRINT("No output interpretation has been provided for this sensor. ");
            PRINT("FIFO data will be printed as hex values. ");
            PRINT("For registering the payload interpretation, use the addse option\r\n");
        }
    }
    else
    {
        ERROR("The requested sensor is not present in the loaded firmware!\r\n");

        return;
    }

    INFO("Sensor ID: %u, sample rate: %f Hz, latency: %lu ms\r\n", sen_id, sample_rate, sen_latency);

    sensor_details = parse_add_sensor_details(sen_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Insufficient parsing slots\r\n");

        return;
    }

    /* If through the logging command, check for valid sample rate before enabling logging */
    if (sample_rate > 0.0f)
    {
        /* Register if not already requested earlier */
        if (sensor_details->parse_flag == PARSE_FLAG_NONE)
        {
            callback = bhy2_get_callback(sen_id);
            BHY2_ASSERT(bhy2_register_fifo_parse_callback(sen_id, callback, parse_table, bhy2));
        }

        sensor_details->parse_flag = PARSE_SET_FLAG(sensor_details->parse_flag, parse_flag);
        sensor_details->scaling_factor = get_sensor_default_scaling(sen_id);
        sensors_active[sen_id] = true;

        /* Flush sensor data from the FIFO */
        BHY2_ASSERT(bhy2_flush_fifo(sen_id, bhy2));

        /* Enable sensor and set sample rate. Disable if there is no source requesting it */
        if ((sample_rate > 0.0f) || (sensor_details->parse_flag == PARSE_FLAG_NONE))
        {
            BHY2_ASSERT(bhy2_set_virt_sensor_cfg(sen_id, sample_rate, sen_latency, bhy2));
        }
    }
    else
    {
        /* Flush sensor data from the FIFO */
        BHY2_ASSERT(bhy2_flush_fifo(sen_id, bhy2));

        /* Enable sensor and set sample rate. Disable if there is no source requesting it */
        if ((sample_rate > 0.0f) || (sensor_details->parse_flag == PARSE_FLAG_NONE))
        {
            BHY2_ASSERT(bhy2_set_virt_sensor_cfg(sen_id, sample_rate, sen_latency, bhy2));
        }

        sensor_details->parse_flag = PARSE_CLEAR_FLAG(sensor_details->parse_flag, parse_flag);

        /* Disable if there is no source requesting it */
        if (sensor_details->parse_flag == PARSE_FLAG_NONE)
        {
            sensors_active[sen_id] = false;
            BHY2_ASSERT(bhy2_deregister_fifo_parse_callback(sen_id, bhy2));
            sensor_details->id = 0;
        }
    }

    /* Sensor data will be parsed and printed after processing all arguments */
}

static void parse_custom_sensor_default(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint8_t this_sensor_id;
    uint8_t this_sensor_payload;
    uint8_t parse_flag;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;

    this_sensor_id = callback_info->sensor_id;
    this_sensor_payload =
        (uint8_t)custom_driver_information[this_sensor_id - BHY2_SENSOR_ID_CUSTOM_START].sensor_payload;

    if (this_sensor_payload > callback_info->data_size)
    {
        ERROR("Mismatch in payload size\r\n");

        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        /* Print sensor ID */
        DATA("SID: %u; T: %lu.%09lu; ", this_sensor_id, s, ns);

        for (uint16_t i = 0; i < this_sensor_payload - 1; i++)
        {
            /* Output raw data in hex */
            PRINT_D("%x ", callback_info->data_ptr[i]);
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

static void parse_custom_sensor(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint8_t idx;
    char *strtok_ptr;
    char *parameter_delimiter = ":";
    char tmp_output_formats[BHY2CLI_MAX_STRING_LENGTH];
    uint8_t rel_sensor_id; /* Relative sensor ID  */
    uint32_t s, ns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

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

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;

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

        return;
    }

    idx = 0;
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        /* Print sensor id and timestamp */
        DATA("%u; %lu.%09lu; ", callback_info->sensor_id, s, ns);

        /* Parse output_formats and output data depending on the individual format of an output */

        while (strtok_ptr != NULL)
        {

            if (strcmp(strtok_ptr, "u8") == 0)
            {
                tmp_u8 = callback_info->data_ptr[idx];
                idx += 1;

                PRINT_D("%u ", tmp_u8);
            }
            else if (strcmp(strtok_ptr, "u16") == 0)
            {
                tmp_u16 = BHY2_LE2U16(&callback_info->data_ptr[idx]);
                idx += 2;

                PRINT_D("%u ", tmp_u16);
            }
            else if (strcmp(strtok_ptr, "u32") == 0)
            {
                tmp_u32 = BHY2_LE2U32(&callback_info->data_ptr[idx]);
                idx += 4;

                PRINT_D("%lu ", tmp_u32);
            }
            else if (strcmp(strtok_ptr, "s8") == 0)
            {
                tmp_s8 = (int8_t)callback_info->data_ptr[idx];
                idx += 1;

                PRINT_D("%d ", tmp_s8);
            }
            else if (strcmp(strtok_ptr, "s16") == 0)
            {
                tmp_s16 = BHY2_LE2S16(&callback_info->data_ptr[idx]);
                idx += 2;

                PRINT_D("%d ", tmp_s16);
            }
            else if (strcmp(strtok_ptr, "s32") == 0)
            {
                tmp_s32 = BHY2_LE2S32(&callback_info->data_ptr[idx]);
                idx += 4;

                PRINT_D("%ld ", tmp_s32);
            }
            else if (strcmp(strtok_ptr, "c") == 0)
            {
                tmp_data_c = callback_info->data_ptr[idx];
                idx += 1;

                PRINT_D("%c ", tmp_data_c);
            }
            else if (strcmp(strtok_ptr, "f") == 0)
            {
                /* Float values have to be read as unsigned and then interpreted as float */
                u32_to_float.data_u32 = BHY2_LE2U32(&callback_info->data_ptr[idx]);
                idx += 4;

                /* The binary data has to be interpreted as a float */
                PRINT_D("%6.4f ", u32_to_float.data_float);
            }

            strtok_ptr = strtok(NULL, parameter_delimiter);
        }

        PRINT_D("\r\n");
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 timestamp,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }

    if (idx != custom_driver_information[rel_sensor_id].sensor_payload)
    {
        ERROR("Provided Output format sizes don't add up to total sensor payload!\r\n");

        return;
    }
}

static void add_sensor(const char *payload, struct bhy2_cli_ref *cli_ref)
{
    struct bhy2_dev *bhy2 = &cli_ref->bhy2;
    struct parse_ref *parse_table = &cli_ref->parse_table;
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

    start = (char *)payload;
    end = strchr(start, ':');

    if (end == NULL)
    {
        ERROR("Add Sensor format error\r\n");

        return;
    }

    /* Parse sensor ID */

    /* Check length of string */
    if (((uint32_t)(end - start)) > BHY2CLI_MAX_STRING_LENGTH)
    {
        ERROR("Too many characters for sensor ID!\r\n");

        return;
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

        return;
    }

    /* Check length of string */
    if (((uint32_t)(end - start)) > BHY2CLI_MAX_STRING_LENGTH)
    {
        ERROR("Too many characters for sensor name. Only %u characters allowed\r\n", BHY2CLI_MAX_STRING_LENGTH);

        return;
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

        return;
    }

    strncpy(str_sensor_payload, start, (size_t)(end - start));
    str_sensor_payload[end - start] = '\0';
    sensor_payload = (uint8_t)strtol(str_sensor_payload, NULL, 10);
    INFO("Sensor Payload: %u \r\n", sensor_payload);

    /* Parse output formats string, final parsing of each output is done in the parsing callback function */
    start = end + 1;
    len_of_output_formats = (uint8_t)strlen(start);
    end = start + len_of_output_formats;
    strncpy(output_formats, start, (size_t)((size_t)(end - start)));
    output_formats[end - start] = '\0';

    /* Get the sensor information */
    BHY2_ASSERT(bhy2_get_sensor_info(sensor_id, &sensor_info, bhy2));

    /* Check if supplied payload matches the event size. Note event size includes sensor id in the payload */
    if (sensor_info.event_size != (sensor_payload + 1))
    {
        ERROR("Provided total payload size of sensor ID %u doesn't match the actual payload size!\r\n", sensor_id);

        return;
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
    BHY2_ASSERT(bhy2_register_fifo_parse_callback(sensor_id, parse_custom_sensor, parse_table, bhy2));

    PRINT("Adding custom driver payload successful\r\n");
}

static void klio_enable(struct bhy2_dev *bhy2)
{
    if (!klio_enabled)
    {
        uint8_t buf[255];
        uint16_t size = sizeof(buf);
        uint32_t klio_driver_status;

        int major = -1;
        int minor = -1;
        int version = -1;
        int count = 0;

        BHY2_ASSERT(bhy2_klio_get_parameter(KLIO_PARAM_ALGORITHM_VERSION, buf, &size, bhy2));
        BHY2_ASSERT(bhy2_klio_read_reset_driver_status(&klio_driver_status, bhy2));

        if (size > 0)
        {
            count = sscanf((char *)buf, "%d.%d.%d", &major, &minor, &version);
        }

        if (major < 0 || minor < 0 || version < 0 || count != 3)
        {
            PRINT("Unable to get Klio firmware version.\r\n");
            BHY2_ASSERT(BHY2_E_MAGIC); /* Invalid firmware error */
        }

        if (major != 3)
        {
            PRINT("The supported Klio firmware is version 3.x.x.\r\n");
            BHY2_ASSERT(BHY2_E_MAGIC); /* Invalid firmware error */
        }

        BHY2_ASSERT(bhy2_klio_get_parameter(KLIO_PARAM_RECOGNITION_MAX_PATTERNS, buf, &size, bhy2));
        BHY2_ASSERT(bhy2_klio_read_reset_driver_status(&klio_driver_status, bhy2));

        memcpy(&klio_vars.max_patterns, buf, 2);

        size = sizeof(buf);

        BHY2_ASSERT(bhy2_klio_get_parameter(KLIO_PARAM_PATTERN_BLOB_SIZE, buf, &size, bhy2));
        BHY2_ASSERT(bhy2_klio_read_reset_driver_status(&klio_driver_status, bhy2));

        memcpy(&klio_vars.max_pattern_blob_size, buf, 2);

        klio_vars.auto_load_pattern_write_index = 0;

        klio_enabled = true;
    }
}

static void klio_status(struct bhy2_dev *bhy2)
{
    uint32_t klio_driver_status;

    BHY2_ASSERT(bhy2_klio_read_reset_driver_status(&klio_driver_status, bhy2));
    INFO("[kstatus] ");
    PRINT("Status: %lu\r\n", klio_driver_status);
}

static void klio_set_state(const char *arg1, const char *arg2, const char *arg3, const char *arg4,
                           struct bhy2_dev *bhy2)
{
    bhy2_klio_sensor_state_t state =
    { .learning_enabled = atoi(arg1), .learning_reset = atoi(arg2), .recognition_enabled = atoi(arg3),
      .recognition_reset = atoi(arg4) };

    INFO("[ksetstate] ");
    PRINT("Learning enabled    : %u\r\n", state.learning_enabled);
    INFO("[ksetstate] ");
    PRINT("Learning reset      : %u\r\n", state.learning_reset);
    INFO("[ksetstate] ");
    PRINT("Recognition enabled : %u\r\n", state.recognition_enabled);
    INFO("[ksetstate] ");
    PRINT("Recognition reset   : %u\r\n", state.recognition_reset);
    BHY2_ASSERT(bhy2_klio_set_state(&state, bhy2));
}

static void klio_get_state(struct bhy2_dev *bhy2)
{
    bhy2_klio_sensor_state_t state;

    BHY2_ASSERT(bhy2_klio_get_state(&state, bhy2));
    INFO("[kgetstate] ");
    PRINT("Learning enabled    : %u\r\n", state.learning_enabled);
    INFO("[kgetstate] ");
    PRINT("Learning reset      : %u\r\n", state.learning_reset);
    INFO("[kgetstate] ");
    PRINT("Recognition enabled : %u\r\n", state.recognition_enabled);
    INFO("[kgetstate] ");
    PRINT("Recognition reset   : %u\r\n", state.recognition_reset);
}

static int32_t hex_to_char(char c)
{
    c = tolower(c);

    if (c >= '0' && c <= '9')
    {
        c -= '0';
    }
    else if (c >= 'a' && c <= 'f')
    {
        c -= 'a' - 10;
    }
    else
    {
        return -1;
    }

    return c;
}

static int32_t pattern_blob_to_bytes(const uint8_t *pattern_blob_char, uint8_t *pattern_blob)
{
    size_t length = strlen((char *)pattern_blob_char);

    for (int i = 0; i < length; i += 2)
    {
        int32_t u = hex_to_char(pattern_blob_char[i]);
        int32_t l = hex_to_char(pattern_blob_char[i + 1]);

        if (u < 0 || l < 0)
        {
            return -1;
        }

        pattern_blob[i / 2] = ((char)u << 4) | (char)l;
    }

    return length / 2;
}

static void klio_load_pattern(const char *arg1, const char *arg2, struct bhy2_dev *bhy2)
{
    uint8_t pattern_data[244];
    uint16_t size = pattern_blob_to_bytes((uint8_t *)arg2, (uint8_t *)pattern_data);
    int index = atoi(arg1);

    if (size <= klio_vars.max_pattern_blob_size)
    {
        if (index < klio_vars.max_patterns)
        {
            BHY2_ASSERT(bhy2_klio_write_pattern(index, pattern_data, size, bhy2));

            if (index >= klio_vars.auto_load_pattern_write_index)
            {
                klio_vars.auto_load_pattern_write_index = index + 1;
            }
        }
        else
        {
            INFO("[kldpatt] ");
            ERROR("Pattern index: %d >= Max patterns %d\r\n", index, klio_vars.max_patterns);
            kldpatt_help(NULL);

            return;
        }
    }
    else
    {
        INFO("[kldpatt] ");
        ERROR("Pattern size: %d != Expected pattern size %d\r\n", size, klio_vars.max_pattern_blob_size);
        kldpatt_help(NULL);

        return;
    }
}

static void klio_get_parameter(const uint8_t *arg, struct bhy2_dev *bhy2)
{
    int param_id = atoi((char *)arg);
    uint8_t buf[255];
    uint16_t size = sizeof(buf);
    float similarity = 0.0f;
    uint16_t max_patterns = 0;

    BHY2_ASSERT(bhy2_klio_get_parameter(param_id, buf, &size, bhy2));

    switch (param_id)
    {
        case KLIO_PARAM_ALGORITHM_VERSION:
            INFO("[kgetparam] ");
            buf[sizeof(buf) - 1] = '\0';
            PRINT("Parameter %d: %s\r\n", param_id, buf);
            break;
        case KLIO_PARAM_RECOGNITION_RESPONSIVNESS:
        case KLIO_PARAM_LEARNING_SIMILARITY_THRESHOLD:
            INFO("[kgetparam] ");
            memcpy(&similarity, buf, 4);
            PRINT("Parameter %d: %f\r\n", param_id, similarity);
            break;
        case KLIO_PARAM_PATTERN_BLOB_SIZE:
        case KLIO_PARAM_RECOGNITION_MAX_PATTERNS:
            INFO("[kgetparam] ");
            memcpy(&max_patterns, buf, 2);
            PRINT("Parameter %d: %u\r\n", param_id, max_patterns);
            break;
        case KLIO_PARAM_LEARNING_IGNORE_INSIG_MOVEMENT:
            INFO("[kgetparam] ");
            PRINT("Parameter %d: %u\r\n", param_id, buf[0]);
            break;
        default:
            for (uint16_t q = 0; q < size; q++)
            {
                if (q % 16 == 0)
                {
                    INFO("[kgetparam] ");
                    PRINT("Parameter %d: ", param_id);
                }

                PRINT("%02x ", buf[q]);
                if (q % 16 == 15 || q == size - 1)
                {
                    PRINT("\r\n");
                }
            }

            break;
    }
}

static void klio_set_parameter(const char *arg1, char *arg2, struct bhy2_dev *bhy2)
{
    int param_id = atoi(arg1);
    char *param_value = arg2;
    float cycle_count;
    uint8_t ignore_insig_movement;

    switch (param_id)
    {
        case KLIO_PARAM_RECOGNITION_RESPONSIVNESS:
            cycle_count = atof(param_value);
            BHY2_ASSERT(bhy2_klio_set_parameter(param_id, &cycle_count, sizeof(cycle_count), bhy2));
            break;
        case KLIO_PARAM_LEARNING_IGNORE_INSIG_MOVEMENT:
            ignore_insig_movement = atoi(param_value);
            BHY2_ASSERT(bhy2_klio_set_parameter(param_id, &ignore_insig_movement, sizeof(ignore_insig_movement), bhy2));
            break;
        default:
            break;
    }
}

static void klio_similarity_score(const uint8_t *arg1, const uint8_t *arg2, struct bhy2_dev *bhy2)
{
    uint8_t first_pattern_data[244] = { 0 }, second_pattern_data[244] = { 0 };
    uint16_t pattern1_size = pattern_blob_to_bytes(arg1, first_pattern_data);
    uint16_t pattern2_size = pattern_blob_to_bytes(arg2, second_pattern_data);

    if (pattern1_size != pattern2_size)
    {
        INFO("[ksimscore] ");
        ERROR("Patterns for similarity calculation differ in size\r\n");
    }
    else
    {
        float similarity;
        BHY2_ASSERT(bhy2_klio_similarity_score(first_pattern_data, second_pattern_data, pattern1_size, &similarity,
                                               bhy2));
        INFO("[ksimscore] ");
        PRINT("Similarity: %f\r\n", similarity);
    }
}

static void klio_similarity_score_multiple(const char *arg1, const char *arg2, struct bhy2_dev *bhy2)
{
    uint8_t index = atoi(arg1);
    char *indexes_str = strdup(arg2), *strtok_ptr;
    uint8_t indexes[klio_vars.max_patterns];
    uint8_t count = 0;
    float similarity[klio_vars.max_patterns];

    strtok_ptr = strtok(indexes_str, ",");
    while (strtok_ptr != NULL)
    {
        indexes[count] = atoi(strtok_ptr);
        count++;
        strtok_ptr = strtok(NULL, ",");
    }

    BHY2_ASSERT(bhy2_klio_similarity_score_multiple(index, indexes, count, similarity, bhy2));

    INFO("[kmsimscore] ");
    PRINT("Using pattern id %d as reference: ", index);
    for (uint32_t i = 0; i < count; i++)
    {
        PRINT("%d:%6f ", indexes[i], similarity[i]);
    }

    PRINT("\r\n");

    free(indexes_str);
}

static void klio_pattern_state_operation(const uint8_t operation, const char *arg1, struct bhy2_dev *bhy2)
{
    uint8_t count = 0;
    uint8_t pattern_states[klio_vars.max_patterns];

    char *sep = ",";
    char *str = strdup(arg1);

    if (str == NULL)
    {
        kenpatt_help(NULL);
        kdispatt_help(NULL);

        return;
    }

    char *token = strtok(str, sep);

    while (token != NULL)
    {
        int32_t index = atoi(token);

        if (index < klio_vars.max_patterns && count < klio_vars.max_patterns)
        {
            pattern_states[count++] = index;
        }
        else
        {
            free(str);
            kenpatt_help(NULL);
            kdispatt_help(NULL);

            return;
        }

        token = strtok(NULL, sep);
    }

    free(str);

    BHY2_ASSERT(bhy2_klio_set_pattern_states(operation, pattern_states, count, bhy2));
}

static void log_data(uint8_t sid, uint64_t tns, uint8_t event_size, uint8_t *event_payload, struct logbin_dev *logdev)
{
    if (logdev && logdev->logfile)
    {
#ifndef PC
        coines_set_led(COINES_LED_GREEN, COINES_LED_STATE_ON);
#endif
        logbin_add_data(sid, tns, event_size, event_payload, logdev);
#ifndef PC
        coines_set_led(COINES_LED_GREEN, COINES_LED_STATE_OFF);
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

static void write_meta_info(struct logbin_dev *log, struct bhy2_dev *bhy2)
{
    logbin_start_meta(log);

    bhy2_update_virtual_sensor_list(bhy2);
#ifndef PC
    coines_set_pin_config(COINES_APP30_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
#endif
    for (uint8_t i = 1; i < BHY2_SENSOR_ID_MAX; i++)
    {
        if (bhy2_is_sensor_available(i, bhy2))
        {
            logbin_add_meta(i,
                            get_sensor_name(i),
                            bhy2->event_size[i] - 1,
                            get_sensor_parse_format(i),
                            get_sensor_axis_names(i),
                            get_sensor_default_scaling(i),
                            log);
        }
    }

    logbin_end_meta(log);

#ifndef PC
    coines_set_pin_config(COINES_APP30_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
#endif
}

static void schema_info(struct bhy2_dev *bhy2)
{
    /* Update virtual sensor */
    bhy2_update_virtual_sensor_list(bhy2);

    /* Get present virtual sensor */
    bhy2_get_virt_sensor_list(bhy2);

    PRINT("Schema List.\r\n");
    PRINT(
        "Sensor ID |                          Sensor Name | Event Size |      Parse Format      |      Axis Names      |  Scaling  |\r\n");
    PRINT(
        "----------+--------------------------------------+------------+------------------------+----------------------+-----------|\r\n");

    for (uint8_t i = 1; i < BHY2_SENSOR_ID_MAX; i++)
    {
        if (bhy2_is_sensor_available(i, bhy2))
        {
            if (i < BHY2_SENSOR_ID_CUSTOM_START)
            {
                PRINT(" %8u | %36s ", i, get_sensor_name(i));
            }
            else
            {
                PRINT(" %8u | %36s ", i, custom_driver_information[i - BHY2_SENSOR_ID_CUSTOM_START].sensor_name);
            }

            PRINT("| %10u | %22s | %20s | %9.4f |\r\n",
                  bhy2->event_size[i] - 1,
                  get_sensor_parse_format(i),
                  get_sensor_axis_names(i),
                  get_sensor_default_scaling(i));
        }
    }
}

int8_t dmode_help(void *ref)
{
    PRINT("  dmode <mode> \r\n");
    PRINT("    \t -Set the Injection <mode>, Real-Time(r)/Step-by-Step(s)/Normal(n)\r\n");
    PRINT("Eg:  dmode s \r\n");

    return CLI_OK;
}

int8_t dmode_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    int8_t rslt = BHY2_OK;
    char injection_mode[20];
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    uint16_t code = 0x00;
    uint32_t actual_length = 0;
    uint8_t status[8] = { 0 };

    /*! Set the Mode */
    switch (argv[1][0])
    {
        case 'n':
            sprintf(injection_mode, "Normal"); /*Normal Mode */
            rslt = bhy2_set_data_injection_mode(BHY2_NORMAL_MODE, &cli_ref->bhy2);
            break;
        case 'r':
            sprintf(injection_mode, "Real-Time"); /*Real-Time Mode */
            rslt = bhy2_set_data_injection_mode(BHY2_REAL_TIME_INJECTION, &cli_ref->bhy2);
            break;
        case 's':
            sprintf(injection_mode, "Step-by-Step"); /*Step-by-Step Mode */
            rslt = bhy2_set_data_injection_mode(BHY2_STEP_BY_STEP_INJECTION, &cli_ref->bhy2);
            break;
        default:
            break;
    }

    if (rslt != BHY2_OK)
    {
        ERROR("Mode switching failed \r\n");

        return rslt;
    }

    /*! Check if the status register is updated and read the status*/
    rslt = bhy2_read_status(&code, status, 8, &actual_length, &cli_ref->bhy2); /*Fix for mode switching not occurring
                                                                                * cleanly */
    if (rslt != BHY2_OK)
    {
        ERROR("Status check failed \r\n");

        return rslt;
    }

    INFO("Switching to %s mode\r\n", injection_mode);

    /*! Clear the buffer */
    memset(injection_mode, 0, sizeof(injection_mode));

    return CLI_OK;
}

int8_t dinject_help(void *ref)
{
    PRINT("  dinject <input_file.txt>\r\n");
    PRINT("    \t -Pass the name of the input file <input_filename.txt>\r\n");
    PRINT("   Eg:  dinject field_log.txt\r\n\r\n");
    PRINT("   For Logging Mode -\r\n");
    PRINT("  Command Flow: attlog outx logse 114:100 dmode s dinject field_log.txt dmode n logse 114:0 detlog outx\r\n");
    PRINT("  For Streaming Mode -\r\n");
    PRINT("  Command Flow: actse 114:100 dmode s dinject field_log.txt dmode n actse 114:0 \r\n\r\n");
    PRINT(
        "  Note:Ensure that the requisite application specific Data Injection firmware is loaded and the sensor is configured prior to executing Data Injection\r\n\r\n");

    return CLI_OK;
}

int8_t dinject_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    int8_t injection_state = DINJECT_IN_PROGRESS;
    uint8_t hex_data;

    memset(&dinject, 0, sizeof(struct data_inject));

    /*! Initialize the Data Injection Structure */
    BHY2_ASSERT(dinject_init((char*)argv[1], &dinject, &cli_ref->bhy2));

    /*    coines_delay_msec(100); / *Added for stability. Maintained for future reference * / */

    while (injection_state == DINJECT_IN_PROGRESS)
    {

        if (get_interrupt_status())
        {
            /*! Data from the FIFO is read and the relevant callback if registered are called */
            BHY2_ASSERT(bhy2_get_and_process_fifo(fifo_buffer, sizeof(fifo_buffer), &cli_ref->bhy2));

            /*            coines_delay_msec(5); / *Added for stability. Maintained for future reference * / */
        }

        /*! Read '1' '8Bit Hex Data' */
        dinject_parse_file(dinject.in_log_ptr, _8BIT_HEX_LEN, 1, &hex_data);

        /*! Data from the Input Log File is injected to relevant virtual sensor driver */
        injection_state = dinject_inject_data(hex_data, &dinject, &cli_ref->bhy2);

        coines_delay_msec(1);
    }

    /*! Close the Log file and reset the buffer parameters */
    BHY2_ASSERT(dinject_deinit(&dinject, &cli_ref->bhy2));

    /*! Clear the FIFO */
    memset(fifo_buffer, 0, sizeof(fifo_buffer)); /*Local Buffer */
    BHY2_ASSERT(bhy2_clear_fifo(0xFF, &cli_ref->bhy2)); /*Read and Flush Wakeup and Non-Wakeup FIFO */
    BHY2_ASSERT(bhy2_clear_fifo(0xFE, &cli_ref->bhy2)); /*Flush all the FIFOs */

    return CLI_OK;
}

int8_t pm_help(void *ref)
{
    PRINT("  -m OR postm <pm_log_filename.bin>\r\n");
    PRINT("    \t -Pass <pm_log_filename.bin> to log the post mortem information\r\n");

    return CLI_OK;
}

int8_t pm_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    struct bhy2_post_mortem post_mortem_data = { 0 };
    int8_t rslt;
    uint8_t error_value = 0, chip_ctrl_value = 0;

    /*! Check the Error Status */
    rslt = bhy2_get_regs(BHY2_REG_ERROR_VALUE, &error_value, 1, &cli_ref->bhy2);

    PRINT("Error Reg Value : %x\r\n", error_value);

    if ((error_value == PM_DATA_AVAILABLE) || (error_value == WATCH_DOG_RESET) || (error_value == FATAL_FIRMWARE_ERROR))
    {
        /*! Get the Post Mortem data */
        rslt = get_post_mortem_data(&post_mortem_data, &cli_ref->bhy2);
        if (rslt != BHY2_OK)
        {
            PRINT("Post Mortem Data Retrieval Failed. Error : %d\r\n", rslt);

            return rslt;
        }

        /*! Log the Post Mortem data */
        rslt = log_post_mortem_data((char *)argv[1], &post_mortem_data, sizeof(struct bhy2_post_mortem));
        if (rslt != PM_LOG_SUCCESS)
        {
            PRINT("Post Mortem Data Logging Failed. Error : %d\r\n", rslt);

            return rslt;
        }

#if PM_DEBUG

        /*! Print the post mortem data */
        rslt = print_post_mortem_data(&post_mortem_data);
        if (rslt != PM_PARSE_SUCCESS)
        {
            PRINT("Post Mortem Data Parsing Failed. \r\n");

            return rslt;
        }

#endif

        memset((uint8_t*)&post_mortem_data, 0, sizeof(struct bhy2_post_mortem));

        /*! Read the Chip Control Register */
        rslt = bhy2_get_regs(BHY2_REG_CHIP_CTRL, &chip_ctrl_value, 1, &cli_ref->bhy2);

        /*! Configure Chip Register to Clear Error and Debug Registers */
        uint8_t clr_err = chip_ctrl_value | BHY2_CHIP_CTRL_CLR_ERR_REG;
        rslt = bhy2_set_regs(BHY2_REG_CHIP_CTRL, &clr_err, 1, &cli_ref->bhy2);
    }
    else
    {
        PRINT("No Fatal error observed. Post Mortem Data not available. \r\n");
    }

    PRINT("\r\n");

    return CLI_OK;
}

int8_t dactse_help(void *ref)
{
    PRINT("  dactse\r\n");
    PRINT("    \t -Deactivates all the active sensors\r\n");

    return CLI_OK;
}

int8_t dactse_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;

    /*! Disable the Sensors */
    bhy2_exit(cli_ref);

    /*! Clear the FIFO */
    memset(fifo_buffer, 0, sizeof(fifo_buffer)); /*Local Buffer */
    BHY2_ASSERT(bhy2_clear_fifo(0xFF, &cli_ref->bhy2)); /*Read and Flush Wakeup and Non-Wakeup FIFO */
    BHY2_ASSERT(bhy2_clear_fifo(0xFE, &cli_ref->bhy2)); /*Flush all the FIFOs */

    PRINT("Deactivated all the Sensors\r\n");

    return CLI_OK;
}

int8_t lsactse_help(void *ref)
{
    PRINT("  lsactse\r\n");
    PRINT("    \t -Lists all the active sensors\r\n");

    return CLI_OK;
}

int8_t lsactse_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    uint8_t act_sensors = 0;

    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    struct bhy2_virt_sensor_conf act_sensor_conf;
    struct parse_sensor_details *sensor_details;

    /*! Check the Sensor status */
    for (uint16_t i = 0; i < 256; i++)
    {
        if (sensors_active[i])
        {
            act_sensors++;
            if (act_sensors == 1)
            {
                PRINT("Active Sensors -\r\n");
            }

            BHY2_ASSERT(bhy2_get_virt_sensor_cfg(i, &act_sensor_conf, &cli_ref->bhy2));

            sensor_details = parse_get_sensor_details(i, &cli_ref->parse_table);
            PRINT("SID : %3d \t ODR : %4.2f \t R : %4d \t Acquisition : %s\r\n",
                  i,
                  act_sensor_conf.sample_rate,
                  act_sensor_conf.range,
                  (sensor_details->parse_flag == PARSE_FLAG_STREAM) ? "Streaming" : "Logging");
        }
    }

    /*! If no active sensors */
    if (act_sensors == 0)
    {
        PRINT("No Active Sensors\r\n");
    }

    if (cli_ref->parse_table.logdev.logfile != NULL)
    {
        PRINT("Attached Log File : %s\r\n", cli_ref->parse_table.logdev.logfilename);
    }
    else
    {
        PRINT("No File attached for Logging\r\n");
    }

    return CLI_OK;
}

int8_t mtapen_help(void *ref)
{
    PRINT("  mtapen <tap-setting>\r\n");
    PRINT("    \t -Enable the Multi Tap Configuration \r\n");
    PRINT("    \t -0 : No Tap \r\n");
    PRINT("    \t -1 : Single Tap \r\n");
    PRINT("    \t -2 : Double Tap \r\n");
    PRINT("    \t -3 : Double Single Tap \r\n");
    PRINT("    \t -4 : Triple Tap \r\n");
    PRINT("    \t -5 : Triple Single Tap \r\n");
    PRINT("    \t -6 : Triple Double Tap \r\n");
    PRINT("    \t -7 : Triple Double Single Tap \r\n");

    return CLI_OK;
}

int8_t mtapen_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    int8_t rslt = BHY2_OK;

    enum bhi3_multi_tap_val multitap_setting = (uint8_t)atoi((char *)argv[1]);

    rslt = bhi3_multi_tap_set_config((uint8_t *)&multitap_setting, &cli_ref->bhy2);

    if (rslt != BHY2_OK)
    {
        ERROR("Multi Tap Parameter Set Failed \r\n");

        return rslt;
    }

    PRINT("Multi Tap Parameter set to  %s\r\n", bhi3_multi_tap_string_out[multitap_setting]);

    return CLI_OK;
}

int8_t mtapinfo_help(void *ref)
{
    PRINT("  mtapinfo \r\n");
    PRINT("    \t -Get the Multi Tap Info\r\n");

    return CLI_OK;
}

int8_t mtapinfo_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    int8_t rslt = BHY2_OK;
    uint16_t len = BHY2_LE24MUL(BHI3_MULTI_TAP_ENABLE_LENGTH);
    uint8_t buffer[len];

    rslt = bhi3_multi_tap_get_config((uint8_t *)buffer, &cli_ref->bhy2);

    if (rslt != BHY2_OK)
    {
        ERROR("Multi Tap Parameter Get Failed \r\n");

        return rslt;
    }

    PRINT("Multi Tap Info : %s\r\n", bhi3_multi_tap_string_out[buffer[0]]);

    return CLI_OK;
}

int8_t mtapsetcnfg_help(void *ref)
{
    PRINT("  mtapsetcnfg <s_cnfg> <d_cnfg> <t_cnfg> \r\n");
    PRINT("    \t -Set the Multi Tap Configurations (in Hex)\r\n");
    PRINT("    \t -<s_cnfg> : Single Tap Configuration\r\n");
    PRINT("    \t\t -<s_cnfg> : <0x00>|<mode|max_pks_for_tap|<wait_for_timeout|axis_sel> [MSB->LSB]\r\n");
    PRINT("    \t\t -<mode> : <Robust/Normal/Sensitive> -> <2/1/0>\r\n");
    PRINT("    \t\t -<max_pks_for_tap> : 6\r\n");
    PRINT("    \t\t -<wait_for_timeout> : <Robust/Normal/Sensitive> -> <2/1/0>\r\n");
    PRINT("    \t\t -<axis_sel> : <Z/Y/X> -> <2/1/0>\r\n");
    PRINT("    \t -<d_cnfg> : Double Tap Configuration\r\n");
    PRINT("    \t\t -<d_cnfg> : <max_ges_dur>|<tap_peak_thrs> [MSB->LSB]\r\n");
    PRINT("    \t\t -<max_ges_dur> : <15:10> -> 0 to 2520ms at resolution of 40ms\r\n");
    PRINT("    \t\t -<tap_peak_thrs> : <9:0> -> 0 to 2000mg at resolution of 1.953mg \r\n");
    PRINT("    \t -<t_cnfg> : Triple Tap Configuration\r\n");
    PRINT(
        "    \t\t -<t_cnfg> : <quite_time_after_ges>|<min_quite_dur_bw_taps>|<tap_shock_settl_dur>|<max_dur_bw_pks> [MSB->LSB]\r\n");
    PRINT("    \t\t -<quite_time_after_ges> : <15:12> -> 0 to 75ms at resolution of 5ms\r\n");
    PRINT("    \t\t -<min_quite_dur_bw_taps> : <11:8> -> 0 to 75ms at resolution of 5ms \r\n");
    PRINT("    \t\t -<tap_shock_settl_dur> : <7:4> -> 0 to 75ms at resolution of 5ms\r\n");
    PRINT("    \t\t -<max_dur_bw_pks> : <3:0> -> 0 to 600ms at resolution of 40ms \r\n");

    return CLI_OK;
}

int8_t mtapsetcnfg_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    int8_t rslt = BHY2_OK;

    bhi3_multi_tap_detector_t multitap_cnfg;

    multitap_cnfg.stap_setting.as_uint16 = string_to_int((char *)argv[1]);
    multitap_cnfg.dtap_setting.as_uint16 = string_to_int((char *)argv[2]);
    multitap_cnfg.ttap_setting.as_uint16 = string_to_int((char *)argv[3]);

    rslt = bhi3_multi_tap_detector_set_config((uint8_t *)&multitap_cnfg, &cli_ref->bhy2);

    if (rslt != BHY2_OK)
    {
        ERROR("Multi Tap Detector Parameter Set Failed \r\n");

        return rslt;
    }

    PRINT("Multi Tap Detector Parameter set successfully \r\n");

    return CLI_OK;
}

int8_t mtapgetcnfg_help(void *ref)
{
    PRINT("  mtapgetcnfg \r\n");
    PRINT("    \t -Get the Multi Tap Configurations\r\n");

    return CLI_OK;
}

int8_t mtapgetcnfg_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    int8_t rslt = BHY2_OK;
    uint16_t len = BHY2_LE24MUL(BHI3_MULTI_TAP_DETECTOR_CONFIG_LENGTH);
    uint8_t buffer[len];
    bhi3_multi_tap_detector_t multitap_cnfg;

    rslt = bhi3_multi_tap_detector_get_config((uint8_t *)buffer, &cli_ref->bhy2);

    if (rslt != BHY2_OK)
    {
        ERROR("Multi Tap Parameter Get Failed \r\n");

        return rslt;
    }

    multitap_cnfg.stap_setting.as_s.axis_sel = (buffer[0] & BHI3_SINGLE_TAP_AXIS_SEL_MASK);
    multitap_cnfg.stap_setting.as_s.wait_for_timeout =
        ((buffer[0] & BHI3_SINGLE_TAP_WAIT_TIMEOUT_MASK) >> BHI3_SINGLE_TAP_WAIT_TIMEOUT_SHIFT);
    multitap_cnfg.stap_setting.as_s.max_peaks_for_tap =
        ((buffer[0] & BHI3_SINGLE_TAP_MAX_PEAKS_FOR_TAP_MASK) >> BHI3_SINGLE_TAP_MAX_PEAKS_FOR_TAP_SHIFT);
    multitap_cnfg.stap_setting.as_s.mode =
        ((buffer[0] & BHI3_SINGLE_TAP_FILTER_MODE_MASK) >> BHI3_SINGLE_TAP_FILTER_MODE_SHIFT);
    multitap_cnfg.dtap_setting.as_s.tap_peak_thres = (BHY2_LE2U16(buffer + 2) & BHI3_DOUBLE_TAP_TAP_PEAK_DUR_MASK);
    multitap_cnfg.dtap_setting.as_s.max_gesture_dur =
        ((BHY2_LE2U16(buffer + 2) & BHI3_DOUBLE_TAP_MAX_GES_DUR_MASK) >> BHI3_DOUBLE_TAP_MAX_GES_DUR_SHIFT);
    multitap_cnfg.ttap_setting.as_s.max_dur_between_peaks = (buffer[4] & BHI3_TRIPLE_TAP_MAX_DUR_BW_PEAKS_MASK);
    multitap_cnfg.ttap_setting.as_s.tap_shock_settling_dur =
        ((buffer[4] & BHI3_TRIPLE_TAP_TAP_SHOCK_SETL_DUR_MASK) >> BHI3_TRIPLE_TAP_TAP_SHOCK_SETL_DUR_SHIFT);
    multitap_cnfg.ttap_setting.as_s.min_quite_dur_between_taps = (buffer[5] & BHI3_TRIPLE_TAP_MIN_QT_DUR_BW_PEAKS_MASK);
    multitap_cnfg.ttap_setting.as_s.quite_time_after_gesture =
        ((buffer[5] & BHI3_TRIPLE_TAP_QT_TM_AFTER_GESTURE_MASK) >> BHI3_TRIPLE_TAP_QT_TM_AFTER_GESTURE_SHIFT);

    PRINT("Single Tap CNFG : 0x%04x\r\n", BHY2_LE2U16(buffer));
    PRINT("    \t\t -<axis_sel> : %d\r\n", multitap_cnfg.stap_setting.as_s.axis_sel);
    PRINT("    \t\t -<wait_for_timeout> : %d\r\n", multitap_cnfg.stap_setting.as_s.wait_for_timeout);
    PRINT("    \t\t -<max_pks_for_tap> : %d\r\n", multitap_cnfg.stap_setting.as_s.max_peaks_for_tap);
    PRINT("    \t\t -<mode> : %d\r\n", multitap_cnfg.stap_setting.as_s.mode);
    PRINT("Double Tap CNFG : 0x%04x\r\n", BHY2_LE2U16(buffer + 2));
    PRINT("    \t\t -<tap_peak_thrs> : %d\r\n", multitap_cnfg.dtap_setting.as_s.tap_peak_thres);
    PRINT("    \t\t -<max_ges_dur> : %d\r\n", multitap_cnfg.dtap_setting.as_s.max_gesture_dur);
    PRINT("Triple Tap CNFG : 0x%04x\r\n", BHY2_LE2U16(buffer + 4));
    PRINT("    \t\t -<max_dur_bw_pks> : %d\r\n", multitap_cnfg.ttap_setting.as_s.max_dur_between_peaks);
    PRINT("    \t\t -<tap_shock_settl_dur> : %d\r\n", multitap_cnfg.ttap_setting.as_s.tap_shock_settling_dur);
    PRINT("    \t\t -<min_quite_dur_bw_taps> : %d\r\n", multitap_cnfg.ttap_setting.as_s.min_quite_dur_between_taps);
    PRINT("    \t\t -<quite_time_after_ges> : %d\r\n", multitap_cnfg.ttap_setting.as_s.quite_time_after_gesture);

    return CLI_OK;
}

int8_t accsetfoc_help(void *ref)
{
    PRINT("  accsetfoc <x> <y> <z> \r\n");
    PRINT("    \t -Set the Accelerometer Fast Offset Calibration (in Hex)\r\n");
    PRINT("    \t -Range of Accelerometer FOC value : -128 to 127 [8Bit Resolution]\r\n");
    PRINT("    \t accsetfoc 0x0072 0x0064 0x0080\r\n");

    return CLI_OK;
}

int8_t accsetfoc_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    struct bhy2_data_xyz accfoc = { 0 };

    accfoc.x = string_to_int((char *)argv[1]);
    accfoc.y = string_to_int((char *)argv[2]);
    accfoc.z = string_to_int((char *)argv[3]);

    BHY2_ASSERT(bhi3_set_acc_foc(&accfoc, &cli_ref->bhy2));

    PRINT("Set the Accelerometer Fast Offset Calibration \r\n");

    return CLI_OK;
}

int8_t accgetfoc_help(void *ref)
{
    PRINT("  accgetfoc \r\n");
    PRINT("    \t -Get the Accelerometer Fast Offset Calibration\r\n");
    PRINT("    \t -Range of Accelerometer FOC value : -128 to 127 [8Bit Resolution]\r\n");

    return CLI_OK;
}

int8_t accgetfoc_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    struct bhy2_data_xyz accfoc = { 0 };

    BHY2_ASSERT(bhi3_get_acc_foc(&accfoc, &cli_ref->bhy2));

    PRINT("Accelerometer Fast Offset Calibration : \r\nx : 0x%04x, y : 0x%04x, z : 0x%04x\r\n", (uint16_t)accfoc.x,
          (uint16_t)accfoc.y, (uint16_t)accfoc.z);

    return CLI_OK;
}

int8_t accsetpwm_help(void *ref)
{
    PRINT("  accsetpwm <power_mode> \r\n");
    PRINT("    \t -Set the Accelerometer Power Mode\r\n");
    PRINT("    \t -'0' corresponds to Normal Mode\r\n");
    PRINT("    \t -'2' corresponds to Low Power Mode\r\n");

    return CLI_OK;
}

int8_t accsetpwm_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    enum bhi3_phy_sensor_acc_power_mode accpwm = 0;

    accpwm = (uint8_t)atoi((char *)argv[1]);
    BHY2_ASSERT(bhi3_set_acc_power_mode((uint8_t *)&accpwm, &cli_ref->bhy2));

    PRINT("Set the Accelerometer Power Mode to %s\r\n", bhi3_phy_sensor_acc_pwm_output[accpwm]);

    return CLI_OK;
}

int8_t accgetpwm_help(void *ref)
{
    PRINT("  accgetpwm \r\n");
    PRINT("    \t -Get the Accelerometer Power Mode\r\n");

    return CLI_OK;
}

int8_t accgetpwm_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    enum bhi3_phy_sensor_acc_power_mode accpwm = 0;

    BHY2_ASSERT(bhi3_get_acc_power_mode((uint8_t *)&accpwm, &cli_ref->bhy2));

    PRINT("Accelerometer Power Mode : %s\r\n", bhi3_phy_sensor_acc_pwm_output[accpwm]);

    return CLI_OK;
}

int8_t gyrosetfoc_help(void *ref)
{
    PRINT("  gyrosetfoc <x> <y> <z> \r\n");
    PRINT("    \t -Set the Gyroscope Fast Offset Calibration (in Hex)\r\n");
    PRINT("    \t -Range of Gyroscope FOC value : -512 to 511 [10Bit Resolution]\r\n");
    PRINT("    \t gyrosetfoc 0x0016 0x00f8 0x080\r\n");

    return CLI_OK;
}

int8_t gyrosetfoc_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    struct bhy2_data_xyz gyrofoc = { 0 };

    gyrofoc.x = string_to_int((char *)argv[1]);
    gyrofoc.y = string_to_int((char *)argv[2]);
    gyrofoc.z = string_to_int((char *)argv[3]);

    BHY2_ASSERT(bhi3_set_gyro_foc(&gyrofoc, &cli_ref->bhy2));

    PRINT("Set the Gyroscope Fast Offset Calibration \r\n");

    return CLI_OK;
}

int8_t gyrogetfoc_help(void *ref)
{
    PRINT("  gyrogetfoc \r\n");
    PRINT("    \t -Get the Gyroscope Fast Offset Calibration\r\n");
    PRINT("    \t -Range of Gyroscope FOC value : -512 to 511 [10Bit Resolution]\r\n");

    return CLI_OK;
}

int8_t gyrogetfoc_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    struct bhy2_data_xyz gyrofoc = { 0 };

    BHY2_ASSERT(bhi3_get_gyro_foc(&gyrofoc, &cli_ref->bhy2));

    PRINT("Gyroscope Fast Offset Calibration : \r\nx : 0x%04x, y : 0x%04x, z : 0x%04x\r\n", (uint16_t)gyrofoc.x,
          (uint16_t)gyrofoc.y, (uint16_t)gyrofoc.z);

    return CLI_OK;
}

int8_t gyrosetois_help(void *ref)
{
    PRINT("  gyrosetois <enable/disable> \r\n");
    PRINT("    \t -Set the Gyroscope OIS state\r\n");
    PRINT("    \t -1/0 : Enable/Disable OIS\r\n");

    return CLI_OK;
}

int8_t gyrosetois_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    uint8_t gyroois = 0;

    gyroois = (uint8_t)atoi((char *)argv[1]);
    BHY2_ASSERT(bhi3_set_gyro_ois(&gyroois, &cli_ref->bhy2));

    PRINT("Gyroscope OIS %s\r\n", (gyroois == BHI3_PHY_GYRO_ENABLE_OIS) ? "Enabled" : "Disabled");

    return CLI_OK;
}

int8_t gyrogetois_help(void *ref)
{
    PRINT("  gyrogetois \r\n");
    PRINT("    \t -Get the Gyroscope OIS status \r\n");

    return CLI_OK;
}

int8_t gyrogetois_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    uint8_t gyroois = 0;

    BHY2_ASSERT(bhi3_get_gyro_ois(&gyroois, &cli_ref->bhy2));

    PRINT("Gyroscope OIS Status : %s\r\n", (gyroois == BHI3_PHY_GYRO_ENABLE_OIS) ? "Enabled" : "Disabled");

    return CLI_OK;
}

int8_t gyrosetfs_help(void *ref)
{
    PRINT("  gyrosetfs <enable/disable> \r\n");
    PRINT("    \t -Set the Gyroscope Fast Startup\r\n");
    PRINT("    \t -1/0 : Enable/Disable Fast startup\r\n");

    return CLI_OK;
}

int8_t gyrosetfs_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    uint8_t gyrofs = 0;

    gyrofs = (uint8_t)atoi((char *)argv[1]);
    BHY2_ASSERT(bhi3_set_gyro_fast_startup(&gyrofs, &cli_ref->bhy2));

    PRINT("Gyroscope Fast Startup %s\r\n", (gyrofs == BHI3_PHY_GYRO_ENABLE_FAST_STARTUP) ? "Enabled" : "Disabled");

    return CLI_OK;
}

int8_t gyrogetfs_help(void *ref)
{
    PRINT("  gyrogetfs \r\n");
    PRINT("    \t -Get the Gyroscope Fast Startup status \r\n");

    return CLI_OK;
}

int8_t gyrogetfs_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    uint8_t gyrofs = 0;

    BHY2_ASSERT(bhi3_get_gyro_fast_startup(&gyrofs, &cli_ref->bhy2));

    PRINT("Gyroscope Fast Startup Status : %s\r\n",
          (gyrofs == BHI3_PHY_GYRO_ENABLE_FAST_STARTUP) ? "Enabled" : "Disabled");

    return CLI_OK;
}

int8_t gyrosetcrt_help(void *ref)
{
    PRINT("  gyrosetcrt \r\n");
    PRINT("    \t -Start Gyroscope CRT\r\n");

    return CLI_OK;
}

int8_t gyrosetcrt_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    uint8_t gyrocrt = 0;

    gyrocrt = BHI3_PHY_GYRO_ENABLE_CRT;
    BHY2_ASSERT(bhi3_set_gyro_crt(&gyrocrt, &cli_ref->bhy2));

    PRINT("Gyroscope CRT %s\r\n", (gyrocrt == BHI3_PHY_GYRO_ENABLE_CRT) ? "Enabled" : "Disabled");

    return CLI_OK;
}

int8_t gyrogetcrt_help(void *ref)
{
    PRINT("  gyrogetcrt \r\n");
    PRINT("    \t -Get the Gyroscope CRT status \r\n");

    return CLI_OK;
}

int8_t gyrogetcrt_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    uint8_t gyrocrt[4] = { 0 };

    BHY2_ASSERT(bhi3_get_gyro_crt(gyrocrt, &cli_ref->bhy2));

    PRINT("Gyroscope CRT Status : %s\r\n", (gyrocrt[0] == BHI3_PHY_GYRO_CRT_STATUS_SUCCESS) ? "Successful" : "Failed");

    return CLI_OK;
}

int8_t gyrosetpwm_help(void *ref)
{
    PRINT("  gyrosetpwm <power_mode> \r\n");
    PRINT("    \t -Set the Gyroscope Power Mode\r\n");
    PRINT("    \t -'0' corresponds to Normal Mode\r\n");
    PRINT("    \t -'1' corresponds to Performance Mode\r\n");
    PRINT("    \t -'2' corresponds to Low Power Mode\r\n");

    return CLI_OK;
}

int8_t gyrosetpwm_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    enum bhi3_phy_sensor_gyro_power_mode gyropwm = 0;

    gyropwm = (uint8_t)atoi((char *)argv[1]);
    BHY2_ASSERT(bhi3_set_gyro_power_mode((uint8_t *)&gyropwm, &cli_ref->bhy2));

    PRINT("Set the Gyroscope Power Mode to %s\r\n", bhi3_phy_sensor_gyro_pwm_output[gyropwm]);

    return CLI_OK;
}

int8_t gyrogetpwm_help(void *ref)
{
    PRINT("  gyrogetpwm \r\n");
    PRINT("    \t -Get the Gyroscope Power Mode\r\n");

    return CLI_OK;
}

int8_t gyrogetpwm_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    enum bhi3_phy_sensor_gyro_power_mode gyropwm = 0;

    BHY2_ASSERT(bhi3_get_gyro_power_mode((uint8_t *)&gyropwm, &cli_ref->bhy2));

    PRINT("Gyroscope Power Mode : %s\r\n", bhi3_phy_sensor_gyro_pwm_output[gyropwm]);

    return CLI_OK;
}

int8_t gyrosettat_help(void *ref)
{
    PRINT("  gyrosettat <enable/disable> \r\n");
    PRINT("    \t -Set the Gyroscope Timer Auto Trim state\r\n");
    PRINT("    \t -1/0 : Enable/Disable Timer Auto Trim\r\n");

    return CLI_OK;
}

int8_t gyrosettat_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    uint8_t gyrotat = 0;

    gyrotat = (uint8_t)atoi((char *)argv[1]);
    BHY2_ASSERT(bhi3_set_gyro_timer_auto_trim(&gyrotat, &cli_ref->bhy2));

    PRINT("Gyroscope Timer Auto Trim %s\r\n",
          (gyrotat == BHI3_PHY_GYRO_ENABLE_TIMER_AUTO_TRIM) ? "Enabled" : "Disabled");

    return CLI_OK;
}

int8_t gyrogettat_help(void *ref)
{
    PRINT("  gyrogettat \r\n");
    PRINT("    \t -Get the Gyroscope Timer Auto Trim status \r\n");

    return CLI_OK;
}

int8_t gyrogettat_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    uint8_t gyrotat = 0;

    BHY2_ASSERT(bhi3_get_gyro_timer_auto_trim(&gyrotat, &cli_ref->bhy2));

    PRINT("Gyroscope Timer Auto Trim Status : %s\r\n",
          (gyrotat == BHI3_PHY_GYRO_ENABLE_TIMER_AUTO_TRIM) ? "Enabled" : "Disabled");

    return CLI_OK;
}

int8_t wwwsetcnfg_help(void *ref)
{
    PRINT("  wwwsetcnfg <maf> <manf> <alr> <all> <apd> <apu> <mdm> <mdq>\r\n");
    PRINT("    \t -Set the Wrist Wear Wakeup Configuration\r\n");
    PRINT("    \t <maf> : min_angle_focus (u16), range 1024 to 1774\r\n");
    PRINT("    \t <manf> : min_angle_nonfocus(u16), range 1448 to 1856\r\n");
    PRINT("    \t <alr> : angle_landscape_right (u8), range 88 to 128\r\n");
    PRINT("    \t <all> : angle_landscape_left(u8), range 88 to 128\r\n");
    PRINT("    \t <apd> : angle_portrait_down (u8), range 0 to 179\r\n");
    PRINT("    \t <apu> : angle_portrait_up(u8), range 222 to 247\r\n");
    PRINT("    \t <mdm> : min_dur_moved (u8), range 1 to 10s, in steps of 20ms\r\n");
    PRINT("    \t <mdq> : min_dur_quite(u8), range 1 to 10s, in steps of 20ms\r\n");

    return CLI_OK;
}

int8_t wwwsetcnfg_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    bhi3_wrist_wear_wakeup_config_param_t www = { 0 };

    www.min_angle_focus = (uint16_t)atoi((char *)argv[1]);
    www.min_angle_nonfocus = (uint16_t)atoi((char *)argv[2]);
    www.angle_landscape_right = (uint8_t)atoi((char *)argv[3]);
    www.angle_landscape_left = (uint8_t)atoi((char *)argv[4]);
    www.angle_portrait_down = (uint8_t)atoi((char *)argv[5]);
    www.angle_portrait_up = (uint8_t)atoi((char *)argv[6]);
    www.min_dur_moved = (uint8_t)atoi((char *)argv[7]);
    www.min_dur_quite = (uint8_t)atoi((char *)argv[8]);

    PRINT("min_angle_focus : %d\r\n", www.min_angle_focus);
    PRINT("min_angle_nonfocus : %d\r\n", www.min_angle_nonfocus);
    PRINT("angle_landscape_right : %d\r\n", www.angle_landscape_right);
    PRINT("angle_landscape_left : %d\r\n", www.angle_landscape_left);
    PRINT("angle_portrait_down : %d\r\n", www.angle_portrait_down);
    PRINT("angle_portrait_up : %d\r\n", www.angle_portrait_up);
    PRINT("min_dur_moved : %d\r\n", www.min_dur_moved);
    PRINT("min_dur_quite : %d\r\n", www.min_dur_quite);

    BHY2_ASSERT(bhi3_set_wrist_wear_wakeup_config((uint8_t *)&www, &cli_ref->bhy2));

    return CLI_OK;
}

int8_t wwwgetcnfg_help(void *ref)
{
    PRINT("  wwwgetcnfg \r\n");
    PRINT("    \t -Get the Wrist Wear Wakeup Configuration\r\n");

    return CLI_OK;
}

int8_t wwwgetcnfg_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    bhi3_wrist_wear_wakeup_config_param_t www = { 0 };

    BHY2_ASSERT(bhi3_get_wrist_wear_wakeup_config((uint8_t *)&www, &cli_ref->bhy2));

    PRINT("min_angle_focus : %d\r\n", www.min_angle_focus);
    PRINT("min_angle_nonfocus : %d\r\n", www.min_angle_nonfocus);
    PRINT("angle_landscape_right : %d\r\n", www.angle_landscape_right);
    PRINT("angle_landscape_left : %d\r\n", www.angle_landscape_left);
    PRINT("angle_portrait_down : %d\r\n", www.angle_portrait_down);
    PRINT("angle_portrait_up : %d\r\n", www.angle_portrait_up);
    PRINT("min_dur_moved : %d\r\n", www.min_dur_moved);
    PRINT("min_dur_quite : %d\r\n", www.min_dur_quite);

    return CLI_OK;
}

int8_t amsetcnfg_help(void *ref)
{
    PRINT("  amsetcnfg <dur> <axis> <thrs> \r\n");
    PRINT("    \t -Set the Any Motion Configuration (in Hex)\r\n");
    PRINT("    \t <dur> : duration (u16), range 0 to 163s in steps of 20ms\r\n");
    PRINT("    \t <axis> : axis select (u8), range 0 to 7, <az>:<ay>:<ax>\r\n");
    PRINT("    \t <thrs> : threshold (u16), range 0 to 1g in steps of 0.5mg\r\n");

    return CLI_OK;
}

int8_t amsetcnfg_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    uint8_t am_cfg[BHI3_PHY_ANY_MOTION_CTRL_LEN];

    uint16_t duration = string_to_int((char *)argv[1]);
    uint8_t axis = (uint8_t)string_to_int((char *)argv[2]);
    uint16_t threshold = string_to_int((char *)argv[3]);

    am_cfg[0] = (duration & 0x00FF);
    am_cfg[1] = (axis << 5) | ((duration >> 8) & 0x1F);
    am_cfg[2] = (threshold & 0x00FF);
    am_cfg[3] = (threshold >> 8);

    BHY2_ASSERT(bhi3_set_anymotion_config(am_cfg, &cli_ref->bhy2));

    PRINT("Any Motion Parameter set successfully \r\n");

    return CLI_OK;
}

int8_t amgetcnfg_help(void *ref)
{
    PRINT("  amgetcnfg \r\n");
    PRINT("    \t -Get the Any Motion Configuration\r\n");

    return CLI_OK;
}

int8_t amgetcnfg_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    uint8_t am_cfg[BHY2_LE24MUL(BHI3_PHY_ANY_MOTION_CTRL_LEN)] = { 0 };
    bhi3_any_no_motion_config_param_t anymotion_cfg = { 0 };

    BHY2_ASSERT(bhi3_get_anymotion_config(am_cfg, &cli_ref->bhy2));

    anymotion_cfg.duration = (uint16_t)(((am_cfg[1]) & 0x1F) | am_cfg[0]);
    anymotion_cfg.axis = (uint8_t)((am_cfg[1]) >> 5);
    anymotion_cfg.threshold = (uint16_t)(((am_cfg[3]) << 8) | am_cfg[2]);

    PRINT("Duration : 0x%02x\r\n", anymotion_cfg.duration);
    PRINT("Axis : 0x%02x\r\n", anymotion_cfg.axis);
    PRINT("Threshold : 0x%02x\r\n", anymotion_cfg.threshold);

    return CLI_OK;
}

int8_t nmsetcnfg_help(void *ref)
{
    PRINT("  nmsetcnfg <dur> <axis> <thrs> \r\n");
    PRINT("    \t -Set the No Motion Configuration (in Hex)\r\n");
    PRINT("    \t <dur> : duration (u16), range 0 to 163s in steps of 20ms\r\n");
    PRINT("    \t <axis> : axis select (u8), range 0 to 7, <az>:<ay>:<ax>\r\n");
    PRINT("    \t <thrs> : threshold (u16), range 0 to 1g in steps of 0.5mg\r\n");

    return CLI_OK;
}

int8_t nmsetcnfg_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    uint8_t nm_cfg[BHI3_PHY_NO_MOTION_CTRL_LEN];

    uint16_t duration = string_to_int((char *)argv[1]);
    uint8_t axis = (uint8_t)string_to_int((char *)argv[2]);
    uint16_t threshold = string_to_int((char *)argv[3]);

    nm_cfg[0] = (duration & 0x00FF);
    nm_cfg[1] = (axis << 5) | ((duration >> 8) & 0x1F);
    nm_cfg[2] = (threshold & 0x00FF);
    nm_cfg[3] = (threshold >> 8);

    BHY2_ASSERT(bhi3_set_nomotion_config(nm_cfg, &cli_ref->bhy2));

    PRINT("No Motion Parameter set successfully \r\n");

    return CLI_OK;
}

int8_t nmgetcnfg_help(void *ref)
{
    PRINT("  nmgetcnfg \r\n");
    PRINT("    \t -Get the No Motion Configuration\r\n");

    return CLI_OK;
}

int8_t nmgetcnfg_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    uint8_t nm_cfg[BHY2_LE24MUL(BHI3_PHY_NO_MOTION_CTRL_LEN)] = { 0 };
    bhi3_any_no_motion_config_param_t nomotion_cfg = { 0 };

    BHY2_ASSERT(bhi3_get_nomotion_config(nm_cfg, &cli_ref->bhy2));

    nomotion_cfg.duration = (uint16_t)(((nm_cfg[1]) & 0x1F) | nm_cfg[0]);
    nomotion_cfg.axis = (uint8_t)((nm_cfg[1]) >> 5);
    nomotion_cfg.threshold = (uint16_t)(((nm_cfg[3]) << 8) | nm_cfg[2]);

    PRINT("Duration : 0x%02x\r\n", nomotion_cfg.duration);
    PRINT("Axis : 0x%02x\r\n", nomotion_cfg.axis);
    PRINT("Threshold : 0x%02x\r\n", nomotion_cfg.threshold);

    return CLI_OK;
}

int8_t wgdsetcnfg_help(void *ref)
{
    PRINT("  wgdsetcnfg <mfpy_th> <mfpz_th> <gx_pos> <gx_neg> <gy_neg> <gz_neg> <fpdc> <lmfc> <mdjp> <dp> \r\n");
    PRINT("    \t -Set the Wrist Wear Wakeup Configuration (in Hex)\r\n");
    PRINT("    \t <mfpy_th> : min_flick_peak_y_threshold (u16), range 0x3E8 to 0x9C4\r\n");
    PRINT("    \t <mfpz_th> : min_flick_peak_z_threshold (u16), range 0x1F4 to 0x5DC\r\n");
    PRINT("    \t <gx_pos> : gravity_bounds_x_pos (u16), range 0x0 to 0x800\r\n");
    PRINT("    \t <gx_neg> : gravity_bounds_x_neg (u16), range 0x0 to 0xFC00\r\n");
    PRINT("    \t <gy_neg> : gravity_bounds_y_neg (u16), range 0x0 to 0xFC3F\r\n");
    PRINT("    \t <gz_neg> : gravity_bounds_z_neg (u16), range 0x800 to 0xF912\r\n");
    PRINT("    \t <fpdc> : flick_peak_decay_coeff (u16), range 0x0 to 0x8000\r\n");
    PRINT("    \t <lmfc> : lp_mean_filter_coeff (u16), range 0x0 to 0x8000\r\n");
    PRINT("    \t <mdjp> : max_duration_jiggle_peaks (u16), range 0xA to 0x19\r\n");
    PRINT("    \t <dp> : device_position (u8), 0 or 1\r\n");

    return CLI_OK;
}

int8_t wgdsetcnfg_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    bhi3_wrist_gesture_detect_config_param_t wgd = { 0 };

    wgd.min_flick_peak_y_threshold = string_to_int((char *)argv[1]);
    wgd.min_flick_peak_z_threshold = string_to_int((char *)argv[2]);
    wgd.gravity_bounds_x_pos = string_to_int((char *)argv[3]);
    wgd.gravity_bounds_x_neg = string_to_int((char *)argv[4]);
    wgd.gravity_bounds_y_neg = string_to_int((char *)argv[5]);
    wgd.gravity_bounds_z_neg = string_to_int((char *)argv[6]);
    wgd.flick_peak_decay_coeff = string_to_int((char *)argv[7]);
    wgd.lp_mean_filter_coeff = string_to_int((char *)argv[8]);
    wgd.max_duration_jiggle_peaks = string_to_int((char *)argv[9]);
    wgd.device_position = (uint8_t)string_to_int((char *)argv[10]);

    BHY2_ASSERT(bhi3_set_wrist_gesture_detect_config((uint8_t *)&wgd, &cli_ref->bhy2));

    PRINT("Wrist Gesture Detector Parameter set successfully \r\n");

    return CLI_OK;
}

int8_t wgdgetcnfg_help(void *ref)
{
    PRINT("  wgdgetcnfg \r\n");
    PRINT("    \t -Get the Wrist Wear Wakeup Configuration\r\n");

    return CLI_OK;
}

int8_t wgdgetcnfg_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    bhi3_wrist_gesture_detect_config_param_t wgd = { 0 };

    BHY2_ASSERT(bhi3_get_wrist_gesture_detect_config((uint8_t *)&wgd, &cli_ref->bhy2));

    PRINT("min_flick_peak_y_threshold : 0x%04x\r\n", wgd.min_flick_peak_y_threshold);
    PRINT("min_flick_peak_z_threshold : 0x%04x\r\n", wgd.min_flick_peak_z_threshold);
    PRINT("gravity_bounds_x_pos : 0x%04x\r\n", wgd.gravity_bounds_x_pos);
    PRINT("gravity_bounds_x_neg : 0x%04x\r\n", wgd.gravity_bounds_x_neg);
    PRINT("gravity_bounds_y_neg : 0x%04x\r\n", wgd.gravity_bounds_y_neg);
    PRINT("gravity_bounds_z_neg : 0x%04x\r\n", wgd.gravity_bounds_z_neg);
    PRINT("flick_peak_decay_coeff : 0x%04x\r\n", wgd.flick_peak_decay_coeff);
    PRINT("lp_mean_filter_coeff : 0x%04x\r\n", wgd.lp_mean_filter_coeff);
    PRINT("max_duration_jiggle_peaks : 0x%04x\r\n", wgd.max_duration_jiggle_peaks);
    PRINT("device_position : 0x%02x\r\n", wgd.device_position);

    return CLI_OK;
}

int8_t hmctrig_help(void *ref)
{
    PRINT("  hmctrig\r\n");
    PRINT("    \t= Trigger Head Misalignment Calibration\r\n");

    return CLI_OK;
}

int8_t hmctrig_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    uint8_t hmc_calib_state[4] = { 0 };

    INFO("Executing %s\r\n", argv[0]);

    hmc_calib_state[0] = BHY2_HEAD_ORI_HMC_TRIGGER_CALIB_SET;
    BHY2_ASSERT(bhy2_head_tracker_trigger_hmc_calibration(hmc_calib_state, &cli_ref->bhy2));

    PRINT("Triggered Head Misalignment Calibration\r\n");

    return CLI_OK;
}

int8_t hmcsetcnfg_help(void *ref)
{
    PRINT("  hmcsetcnfg <sp_max_dur> <sp_min_dur> <sp_max_samples> <acc_diff_th> \r\n");
    PRINT("    \t -Set the Head Misalignment Configuration (in Hex)\r\n");
    PRINT(
        "    \t <sp_max_dur> : maximum still phase duration required for pitch and roll calibration (u8), resolution 1 sec/LSB\r\n");
    PRINT(
        "    \t <sp_min_dur> : minimum still phase duration required for pitch and roll calibration (u8), resolution 1 sec/LSB\r\n");
    PRINT(
        "    \t <sp_max_samples> : maximal number of samples for still phase for the dynamic part of head misalignment calibration (u8), resolution 1 sample/LSB\r\n");
    PRINT(
        "    \t <acc_diff_th> : threshold of acceleration difference to detect motion in acceleration signal, (i32), resolution 1 mg/LSB [includes 1 bit sign, 8 bits exponent and 23 bits fraction]\r\n");
    PRINT("    \t eg. hmcsetcnfg 0x06 0x02 0x32 0x00002042 \r\n");

    return CLI_OK;
}

int8_t hmcsetcnfg_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    struct bhy2_head_tracker_misalignment_config hmc_config = { 0 };

    hmc_config.still_phase_max_dur = string_to_int((char *)argv[1]);
    hmc_config.still_phase_min_dur = string_to_int((char *)argv[2]);
    hmc_config.still_phase_max_samples = string_to_int((char *)argv[3]);
    hmc_config.acc_diff_threshold = string_to_int((char *)argv[4]);

    BHY2_ASSERT(bhy2_head_tracker_set_hmc_configuration((uint8_t *)&hmc_config, &cli_ref->bhy2));

    PRINT("Head Misalignment Configuration set successfully \r\n");

    return CLI_OK;
}

int8_t hmcgetcnfg_help(void *ref)
{
    PRINT("  hmcgetcnfg \r\n");
    PRINT("    \t -Get the Head Misalignment Configuration\r\n");

    return CLI_OK;
}

int8_t hmcgetcnfg_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    struct bhy2_head_tracker_misalignment_config hmc_config = { 0 };

    BHY2_ASSERT(bhy2_head_tracker_get_hmc_configuration((uint8_t *)&hmc_config, &cli_ref->bhy2));

    PRINT("still_phase_max_dur : 0x%02x\r\n", hmc_config.still_phase_max_dur);
    PRINT("still_phase_min_dur : 0x%02x\r\n", hmc_config.still_phase_min_dur);
    PRINT("still_phase_max_samples : 0x%02x\r\n", hmc_config.still_phase_max_samples);
    PRINT("acc_diff_threshold : 0x%08x\r\n", hmc_config.acc_diff_threshold);

    return CLI_OK;
}

int8_t hmcsetdefcnfg_help(void *ref)
{
    PRINT("  hmcsetdefcnfg\r\n");
    PRINT("    \t= Set Default Head Misalignment Configuration\r\n");

    return CLI_OK;
}

int8_t hmcsetdefcnfg_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    uint8_t hmcsetdefcnfg_state[4] = { 0 };

    INFO("Executing %s\r\n", argv[0]);

    hmcsetdefcnfg_state[0] = BHY2_HEAD_ORI_HMC_SET_DEF_CONFIG_SET;
    BHY2_ASSERT(bhy2_head_tracker_set_default_hmc_configuration(hmcsetdefcnfg_state, &cli_ref->bhy2));

    PRINT("Set Default Head Misalignment Calibration successfully\r\n");

    return CLI_OK;
}

int8_t hmcver_help(void *ref)
{
    PRINT("  hmcver \r\n");
    PRINT("    \t= Get the Head Misalignment Calibrator version.\r\n");

    return CLI_OK;
}

int8_t hmcver_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    struct bhy2_head_tracker_ver hmc_version;

    INFO("Executing %\r\n", argv[0]);

    BHY2_ASSERT(bhy2_head_tracker_get_hmc_version(&hmc_version, &cli_ref->bhy2));

    PRINT("Head Misalignment Calibrator version: %u.%u.%u\r\n\r\n",
          hmc_version.major,
          hmc_version.minor,
          hmc_version.patch);

    return CLI_OK;
}

int8_t hmcsetcalcorrq_help(void *ref)
{
    PRINT("  hmcsetcalcorrq <quat_x> <quat_y> <quat_z> <quat_w> \r\n");
    PRINT("    \t= Set the Head Misalignment Quaternion Calibration Correction\r\n");
    PRINT(
        "    \t <quat_x> : quaternion x, (i32), resolution 1 mg/LSB [includes 1 bit sign, 8 bits exponent and 23 bits fraction]\r\n");
    PRINT(
        "    \t <quat_y> : quaternion y, (i32), resolution 1 mg/LSB [includes 1 bit sign, 8 bits exponent and 23 bits fraction]\r\n");
    PRINT(
        "    \t <quat_z> : quaternion z, (i32), resolution 1 mg/LSB [includes 1 bit sign, 8 bits exponent and 23 bits fraction]\r\n");
    PRINT(
        "    \t <quat_w> : quaternion w, (i32), resolution 1 mg/LSB [includes 1 bit sign, 8 bits exponent and 23 bits fraction]\r\n");
    PRINT("    \t eg. hmcsetcalcorrq 0x00000600 0x00000002 0x00320000 0x3f000000 \r\n");

    return CLI_OK;
}

int8_t hmcsetcalcorrq_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    struct bhy2_head_tracker_misalignment_quat_corr hmc_quat_corr = { 0 };

    INFO("Executing %s\r\n", argv[0]);

    hmc_quat_corr.quaternion_x = string_to_int((char *)argv[1]);
    hmc_quat_corr.quaternion_y = string_to_int((char *)argv[2]);
    hmc_quat_corr.quaternion_z = string_to_int((char *)argv[3]);
    hmc_quat_corr.quaternion_w = string_to_int((char *)argv[4]);
    hmc_quat_corr.accuracy = 0;
    BHY2_ASSERT(bhy2_head_tracker_set_hmc_quat_calib_corr_config(&hmc_quat_corr, &cli_ref->bhy2));

    PRINT("Head Misalignment Quaternion Calibration Correction set successfully\r\n");

    return CLI_OK;
}

int8_t hmcgetcalcorrq_help(void *ref)
{
    PRINT("  hmcgetcalcorrq \r\n");
    PRINT("    \t= Get the Head Misalignment Quaternion Calibration Correction\r\n");

    return CLI_OK;
}

int8_t hmcgetcalcorrq_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    struct bhy2_head_tracker_misalignment_quat_corr hmc_quat_corr = { 0 };

    BHY2_ASSERT(bhy2_head_tracker_get_hmc_quat_calib_corr_config((uint8_t *)&hmc_quat_corr, &cli_ref->bhy2));

    PRINT("quaternion_x : 0x%08x\r\n", hmc_quat_corr.quaternion_x);
    PRINT("quaternion_y : 0x%08x\r\n", hmc_quat_corr.quaternion_y);
    PRINT("quaternion_z : 0x%08x\r\n", hmc_quat_corr.quaternion_z);
    PRINT("quaternion_w : 0x%08x\r\n", hmc_quat_corr.quaternion_w);
    PRINT("accuracy : 0x%08x\r\n", hmc_quat_corr.accuracy);

    return CLI_OK;
}

int8_t hosetheadcorrq_help(void *ref)
{
    PRINT("  hosetheadcorrq\r\n");
    PRINT("    \t= Set Initial Heading Correction, only for IMU Head Orientation Quaternion\r\n");
    PRINT("    \t -1/0 : Enable/Disable Initial Heading Correction [Quaternion] \r\n");

    return CLI_OK;
}

int8_t hosetheadcorrq_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    uint8_t ho_quat_head_corr_state[4] = { 0 };

    INFO("Executing %s\r\n", argv[0]);

    ho_quat_head_corr_state[0] = string_to_int((char *)argv[1]);
    BHY2_ASSERT(bhy2_head_tracker_set_quat_initial_head_correction(ho_quat_head_corr_state, &cli_ref->bhy2));

    PRINT("Quaternion Initial Heading Correction %s\r\n",
          (ho_quat_head_corr_state[0] == BHY2_HEAD_ORI_QUAT_INITIAL_HEAD_CORR_ENABLE) ? "Enabled" : "Disabled");

    return CLI_OK;
}

int8_t hogetheadcorrq_help(void *ref)
{
    PRINT("  hogetheadcorrq \r\n");
    PRINT("    \t= Get Initial Heading Correction, only for IMU Head Orientation Quaternion\r\n");

    return CLI_OK;
}

int8_t hogetheadcorrq_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    uint8_t ho_quat_head_corr_state[4] = { 0 };

    BHY2_ASSERT(bhy2_head_tracker_get_quat_initial_head_correction(ho_quat_head_corr_state, &cli_ref->bhy2));

    PRINT("Quaternion Initial Heading Correction Status : %s\r\n",
          (ho_quat_head_corr_state[0] == BHY2_HEAD_ORI_QUAT_INITIAL_HEAD_CORR_ENABLE) ? "Enabled" : "Disabled");

    return CLI_OK;
}

int8_t hover_help(void *ref)
{
    PRINT("  hover \r\n");
    PRINT("    \t= Get IMU/NDOF Head Orientation Version \r\n");

    return CLI_OK;
}

int8_t hover_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    struct bhy2_head_tracker_ver ho_version;

    INFO("Executing %\r\n", argv[0]);

    BHY2_ASSERT(bhy2_head_tracker_get_ho_version(&ho_version, &cli_ref->bhy2));

    PRINT("IMU/NDOF Head Orientation version: %u.%u.%u\r\n\r\n", ho_version.major, ho_version.minor, ho_version.patch);

    return CLI_OK;
}

int8_t hosetheadcorre_help(void *ref)
{
    PRINT("  hosetheadcorre\r\n");
    PRINT("    \t= Set Initial Heading Correction, only for IMU Head Orientation Euler\r\n");
    PRINT("    \t -1/0 : Enable/Disable Initial Heading Correction [Euler] \r\n");

    return CLI_OK;
}

int8_t hosetheadcorre_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    uint8_t ho_eul_head_corr_state[4] = { 0 };

    INFO("Executing %s\r\n", argv[0]);

    ho_eul_head_corr_state[0] = string_to_int((char *)argv[1]);
    BHY2_ASSERT(bhy2_head_tracker_set_eul_initial_head_correction(ho_eul_head_corr_state, &cli_ref->bhy2));

    PRINT("Euler Initial Heading Correction %s\r\n",
          (ho_eul_head_corr_state[0] == BHY2_HEAD_ORI_EUL_INITIAL_HEAD_CORR_ENABLE) ? "Enabled" : "Disabled");

    return CLI_OK;
}

int8_t hogetheadcorre_help(void *ref)
{
    PRINT("  hogetheadcorre \r\n");
    PRINT("    \t= Get Initial Heading Correction, only for IMU Head Orientation Euler\r\n");

    return CLI_OK;
}

int8_t hogetheadcorre_callback(uint8_t argc, uint8_t *argv[], void *ref)
{
    struct bhy2_cli_ref *cli_ref = (struct bhy2_cli_ref *)ref;
    uint8_t ho_eul_head_corr_state[4] = { 0 };

    BHY2_ASSERT(bhy2_head_tracker_get_eul_initial_head_correction(ho_eul_head_corr_state, &cli_ref->bhy2));

    PRINT("Euler Initial Heading Correction Status : %s\r\n",
          (ho_eul_head_corr_state[0] == BHY2_HEAD_ORI_EUL_INITIAL_HEAD_CORR_ENABLE) ? "Enabled" : "Disabled");

    return CLI_OK;
}
