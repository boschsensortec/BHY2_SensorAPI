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
* @file       bhy2_pdr_defs.h
* @date       2022-10-17
* @version    v1.4.1
*
*/

#ifndef __BHY2_PDR_DEFS_H__
#define __BHY2_PDR_DEFS_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#define BHY2_SENSOR_ID_PDR             UINT8_C(113)
#define BHY2_SENSOR_ID_PDR_LOG         UINT8_C(119)

#define BHY2_PDR_PAGE                  UINT16_C(10)
#define BHY2_PDR_PARAM(id)             (((BHY2_PDR_PAGE) << 8) | (id))

#define BHY2_PDR_PARAM_FULL_RESET      UINT8_C(1)
#define BHY2_PDR_PARAM_POS_RESET       UINT8_C(2)
#define BHY2_PDR_PARAM_REF_HEAD_DELTA  UINT8_C(32)
#define BHY2_PDR_PARAM_STEP_LENGTH     UINT8_C(46)
#define BHY2_PDR_PARAM_HANDEDNESS      UINT8_C(64)
#define BHY2_PDR_PARAM_DRIVER_VER      UINT8_C(66)
#define BHY2_PDR_PARAM_ALGO_VER        UINT8_C(67)
#define BHY2_PDR_PARAM_PDR_VARIANT     UINT8_C(68)
#define BHY2_PDR_PARAM_DEVICE_POS      UINT8_C(69)

#define BHY2_PDR_VAR_6DOF              UINT8_C(0x00)
#define BHY2_PDR_VAR_9DOF              UINT8_C(0x01)

#define BHY2_PDR_DEV_POS_WRIST         UINT8_C(0x00)
#define BHY2_PDR_DEV_POS_HEAD          UINT8_C(0x01)
#define BHY2_PDR_DEV_POS_SHOE          UINT8_C(0x02)
#define BHY2_PDR_DEV_POS_BACKPACK      UINT8_C(0x03)

struct bhy2_pdr_frame /* Description; Scaling and format; Range; Comment */
{
    int32_t pos_x; /* Position X; 0.001m/LSB 24 bit signed int; -8388.. 8388 m; relative to start position */
    int32_t pos_y; /* Position Y; 0.001m/LSB 24 bit signed int; -8388.. 8388 m; relative to start position */
    int16_t hor_acc; /* Horizontal 95% accuracy distribution; 0.1m/LSB 16 bit signed int; 0 .. 3276.7 m; The horizontal
                      * accuracy is given as radius of a circle, in which the user position is with 95% probability*/
    uint16_t heading; /* Heading; 0.1 degree/LSB 16 bit unsigned int; 0 .. 359.9 degree; Walking direction, measured
                       * clockwise from North */
    uint16_t heading_acc; /* Heading accuracy distribution; 0.1 degree/LSB 16 bit unsigned int; 0 .. 180.0 degree; */
    uint16_t step_count; /* Step count; 1 step / LSB 16 bit unsigned int; 0 .. 65535 steps; */
    uint8_t status; /* Status/Flags; Bit field; ; Bits: 0: full reset, 1: track reset. The bits are set for only the
                     * frame immediately after the reset.*/
};

struct bhy2_pdr_ver
{
    uint8_t major, minor, patch;
    uint16_t fw_build;
};

/* @brief bhy2 pdr sensor log frame*/
typedef struct
{
    float timestamp;
    float accel[3];
    float gyro[3];
} BHY2_PACKED bhy2_pdr_log_frame_t;

#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHY2_PDR_DEFS_H__ */
