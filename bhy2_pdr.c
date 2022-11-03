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
* @file       bhy2_pdr.c
* @date       2022-10-17
* @version    v1.4.1
*
*/

/*********************************************************************/
/* system header files */
#include <string.h>
#include <stdio.h>

/*********************************************************************/
/* BHY2 SensorAPI header files */
#include "bhy2.h"

/*********************************************************************/
/* own header files */
#include "bhy2_pdr.h"

void bhy2_pdr_parse_frame(const uint8_t *data, struct bhy2_pdr_frame *pdr_frame)
{
    if (data && pdr_frame)
    {
        pdr_frame->pos_x = BHY2_LE2S24(&data[0]);
        pdr_frame->pos_y = BHY2_LE2S24(&data[3]);
        pdr_frame->hor_acc = BHY2_LE2S16(&data[6]);
        pdr_frame->heading = BHY2_LE2U16(&data[8]);
        pdr_frame->heading_acc = BHY2_LE2U16(&data[10]);
        pdr_frame->step_count = BHY2_LE2U16(&data[12]);
        pdr_frame->status = data[14];
    }
}

int8_t bhy2_pdr_reset_full(struct bhy2_dev *dev)
{
    uint8_t full_reset[4] = { 1, 0, 0, 0 };

    return bhy2_set_parameter(BHY2_PDR_PARAM(BHY2_PDR_PARAM_FULL_RESET), full_reset, sizeof(full_reset), dev);
}

int8_t bhy2_pdr_reset_position(struct bhy2_dev *dev)
{
    uint8_t pos_reset[4] = { 1, 0, 0, 0 };

    return bhy2_set_parameter(BHY2_PDR_PARAM(BHY2_PDR_PARAM_POS_RESET), pos_reset, sizeof(pos_reset), dev);
}

int8_t bhy2_pdr_set_ref_heading_del(float heading, struct bhy2_dev *dev)
{
    uint8_t ref_heading[4] = { 0 };
    uint16_t heading_delta;

    if (heading >= 0.0f)
    {
        heading_delta = (uint16_t)(heading * 10.0f); /* 0.1 degree/LSB */
    }
    else
    {
        heading_delta = 0;
    }

    if (heading_delta > 3599)
    {
        heading_delta = 3599;
    }

    ref_heading[0] = heading_delta & 0x00FF;
    ref_heading[1] = (heading_delta & 0xFF00) >> 8;

    return bhy2_set_parameter(BHY2_PDR_PARAM(BHY2_PDR_PARAM_REF_HEAD_DELTA), ref_heading, sizeof(ref_heading), dev);
}

int8_t bhy2_pdr_set_step_info(float step_length, float step_length_acc, struct bhy2_dev *dev)
{
    uint8_t stp_len[4] = { 0 };
    uint16_t length = (uint16_t)(step_length * 100.0f); /* 0.01m / LSB */
    uint16_t length_acc = (uint16_t)(step_length_acc * 100.0f);

    if (length > 500)
    {
        length = 500;
    }

    if (length_acc > 500)
    {
        length_acc = 500;
    }

    stp_len[0] = length & 0x00FF;
    stp_len[1] = (length & 0xFF00) >> 8;
    stp_len[2] = length_acc & 0x00FF;
    stp_len[3] = (length_acc & 0xFF00) >> 8;

    return bhy2_set_parameter(BHY2_PDR_PARAM(BHY2_PDR_PARAM_STEP_LENGTH), stp_len, sizeof(stp_len), dev);
}

int8_t bhy2_pdr_set_hand(uint8_t right_hand, struct bhy2_dev *dev)
{
    uint8_t handedness[4] = { 0 };

    if (right_hand)
    {
        handedness[0] = 1;
    }

    return bhy2_set_parameter(BHY2_PDR_PARAM(BHY2_PDR_PARAM_HANDEDNESS), handedness, sizeof(handedness), dev);
}

int8_t bhy2_pdr_get_driver_version(struct bhy2_pdr_ver *version, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint8_t ver[8] = { 0 };
    uint32_t act_len;

    if (version == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_get_parameter(BHY2_PDR_PARAM(BHY2_PDR_PARAM_DRIVER_VER), ver, sizeof(ver), &act_len, dev);

        if (rslt == BHY2_OK)
        {
            version->major = ver[0];
            version->minor = ver[1];
            version->patch = ver[2];
            version->fw_build = BHY2_LE2U16(&ver[3]);
        }
    }

    return rslt;
}

int8_t bhy2_pdr_get_algo_version(struct bhy2_pdr_ver *version, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint8_t ver[8] = { 0 };
    uint32_t act_len;

    if (version == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_get_parameter(BHY2_PDR_PARAM(BHY2_PDR_PARAM_ALGO_VER), ver, sizeof(ver), &act_len, dev);

        if (rslt == BHY2_OK)
        {
            version->major = ver[0];
            version->minor = ver[1];
            version->patch = ver[2];
            version->fw_build = BHY2_LE2U16(&ver[3]);
        }
    }

    return rslt;
}

int8_t bhy2_pdr_get_pdr_variant(uint8_t *variant, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint8_t ver[4] = { 0 };
    uint32_t act_len;

    if (variant == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_get_parameter(BHY2_PDR_PARAM(BHY2_PDR_PARAM_PDR_VARIANT), ver, sizeof(ver), &act_len, dev);

        if (rslt == BHY2_OK)
        {
            *variant = ver[0];
        }
    }

    return rslt;
}

int8_t bhy2_pdr_get_device_position(uint8_t *dev_pos, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint8_t ver[4] = { 0 };
    uint32_t act_len;

    if (dev_pos == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {

        rslt = bhy2_get_parameter(BHY2_PDR_PARAM(BHY2_PDR_PARAM_DEVICE_POS), ver, sizeof(ver), &act_len, dev);

        if (rslt == BHY2_OK)
        {
            *dev_pos = ver[0];
        }
    }

    return rslt;
}
