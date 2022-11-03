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
* @file       bhy2_pdr.h
* @date       2022-10-17
* @version    v1.4.1
*
*/

#ifndef __BHY2_PDR_H__
#define __BHY2_PDR_H__

#include <stdint.h>
#include <stdlib.h>

#include "bhy2.h"
#include "bhy2_pdr_defs.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

void bhy2_pdr_parse_frame(const uint8_t *data, struct bhy2_pdr_frame *pdr_frame);

int8_t bhy2_pdr_reset_full(struct bhy2_dev *dev);

int8_t bhy2_pdr_reset_position(struct bhy2_dev *dev);

int8_t bhy2_pdr_set_ref_heading_del(float heading, struct bhy2_dev *dev);

int8_t bhy2_pdr_set_step_info(float step_length, float step_length_acc, struct bhy2_dev *dev);

int8_t bhy2_pdr_set_hand(uint8_t right_hand, struct bhy2_dev *dev);

int8_t bhy2_pdr_get_driver_version(struct bhy2_pdr_ver *version, struct bhy2_dev *dev);

int8_t bhy2_pdr_get_algo_version(struct bhy2_pdr_ver *version, struct bhy2_dev *dev);

int8_t bhy2_pdr_get_pdr_variant(uint8_t *variant, struct bhy2_dev *dev);

int8_t bhy2_pdr_get_device_position(uint8_t *dev_pos, struct bhy2_dev *dev);

#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif
