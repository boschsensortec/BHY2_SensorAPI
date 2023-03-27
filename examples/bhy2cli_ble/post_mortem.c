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
 * @file    post_mortem.c
 * @brief   Source file for post mortem data retrieval driver
 *
 */

#include "post_mortem.h"
#include <verbose.h>
#include <bhy2_hif.h>

/**
* @brief Function to get the Post Mortem data
*/
int8_t get_post_mortem_data(struct bhy2_post_mortem *pminfo, struct bhy2_dev *bhy2)
{
    uint32_t pmlen = 0;
    int8_t rslt;

    rslt = bhy2_get_post_mortem_data((uint8_t*)pminfo, sizeof(struct bhy2_post_mortem), &pmlen, bhy2);

    return rslt;
}

/**
* @brief Function to log the Post Mortem data
*/
int8_t log_post_mortem_data(const char *pmfilename, struct bhy2_post_mortem *pminfo, uint32_t pmlen)
{
    FILE *pmfile = fopen(pmfilename, "wb");
    size_t pm_data_len = 0;

    if (pmfile)
    {
        pm_data_len = fwrite(pminfo, 1, sizeof(struct bhy2_post_mortem), pmfile);
        if (pm_data_len != sizeof(struct bhy2_post_mortem))
        {
            return PM_LOG_FAILED;
        }

        fclose(pmfile);
        INFO("Saved %d post mortem bytes to %s", pmlen, pmfilename);
    }

    return PM_LOG_SUCCESS;
}

/**
* @brief Function to read the Post Mortem data from the log
*/
int8_t read_post_mortem_data_from_log(const char *pmfilename, struct bhy2_post_mortem *pminfo)
{
    FILE *pmfile = fopen(pmfilename, "r");
    size_t pm_data_len = 0;

    if (pmfile)
    {
        pm_data_len = fread(pminfo, 1, sizeof(struct bhy2_post_mortem), pmfile);
        if (pm_data_len != sizeof(struct bhy2_post_mortem))
        {
            return PM_PARSE_FAILED;
        }

        fclose(pmfile);
        INFO("Read %d post mortem bytes from %s", sizeof(struct bhy2_post_mortem), pmfilename);
    }

    return PM_PARSE_SUCCESS;
}

#if PM_DEBUG

/**
* @brief Function to print the Post Mortem status
*/
int8_t get_post_mortem_status(struct bhy2_post_mortem *pminfo)
{
    if (pminfo != NULL)
    {
        PRINT("POST MORTEM STATUS : \r\n");
        PRINT("valid\t\t:\t0x%0.8x\r\n", pminfo->valid);
        PRINT("flags\t\t:\t0x%0.8x\r\n", pminfo->flags);
        PRINT("\r\n");
    }
    else
    {
        PRINT("Structure in not populated!\n");

        return PM_PARSE_FAILED;
    }

    return PM_PARSE_SUCCESS;
}

/**
* @brief Function to print the Context info
*/
int8_t get_post_mortem_context(struct bhy2_post_mortem *pminfo)
{
    int i = 0;

    if (pminfo != NULL)
    {
        PRINT("CONTEXT : \r\n");
        for (i = 0; i < 26; i++)
        {
            PRINT("reg_%d\t\t:\t0x%0.8x\r\n", i + 1, pminfo->reg[i]);
        }

        PRINT("gp    [reg_%d]\t:\t0x%0.8x\r\n", i + 1, pminfo->reg[i]);
        i++;
        PRINT("fp    [reg_%d]\t:\t0x%0.8x\r\n", i + 1, pminfo->reg[i]);
        i++;
        PRINT("sp    [reg_%d]\t:\t0x%0.8x\r\n", i + 1, pminfo->reg[i]);
        i++;
        PRINT("ilink [reg_%d]\t:\t0x%0.8x\r\n", i + 1, pminfo->reg[i]);
        i++;
        PRINT("reg_%d\t\t:\t0x%0.8x\r\n", i + 1, pminfo->reg[i]);
        i++;
        PRINT("blink [reg_%d]\t:\t0x%0.8x\r\n", i + 1, pminfo->reg[i]);
        PRINT("\r\n");
    }
    else
    {
        PRINT("Structure in not populated!\n");

        return PM_PARSE_FAILED;
    }

    return PM_PARSE_SUCCESS;
}

/**
* @brief Function to print the System status
*/
int8_t get_post_mortem_system_status(struct bhy2_post_mortem *pminfo)
{
    if (pminfo != NULL)
    {
        PRINT("SYSTEM STATUS : \r\n");
        PRINT("eret\t\t:\t0x%0.8x\r\n", pminfo->eret);
        PRINT("erbta\t\t:\t0x%0.8x\r\n", pminfo->erbta);
        PRINT("erstatus\t:\t0x%0.8x\r\n", pminfo->erstatus);
        PRINT("ecr\t\t:\t0x%0.8x\r\n", pminfo->ecr);
        PRINT("efa\t\t:\t0x%0.8x\r\n", pminfo->efa);
        PRINT("icause\t\t:\t0x%0.8x\r\n", pminfo->icause);
        PRINT("mpu_ecr\t\t:\t0x%0.8x\r\n", pminfo->mpu_ecr);
        PRINT("\r\n");
    }
    else
    {
        PRINT("Structure in not populated!\n");

        return PM_PARSE_FAILED;
    }

    return PM_PARSE_SUCCESS;
}

/**
* @brief Function to print the Diagnostic status
*/
int8_t get_post_mortem_diagnostic(struct bhy2_post_mortem *pminfo)
{
    if (pminfo != NULL)
    {
        PRINT("DIAGNOSTIC : \r\n");
        PRINT("diagnostic\t:\t0x%0.8x\r\n", pminfo->diagnostic);
        PRINT("debug state\t:\t0x%0.8x\r\n", pminfo->debugState);
        PRINT("debug val\t:\t0x%0.8x\r\n", pminfo->debugValue);
        PRINT("error val\t:\t0x%0.8x\r\n", pminfo->errorReason);
        PRINT("interrupt\t:\t0x%0.8x\r\n", pminfo->hostIntCtrl);
        PRINT("err report\t:\t0x%0.8x\r\n", pminfo->errorReport);
        PRINT("\r\n");
    }
    else
    {
        PRINT("Structure in not populated!\n");

        return PM_PARSE_FAILED;
    }

    return PM_PARSE_SUCCESS;
}

/**
* @brief Function to print the Stack info
*/
int8_t get_post_mortem_stack_info(struct bhy2_post_mortem *pminfo)
{
    if (pminfo != NULL)
    {
        PRINT("STACK INFO : \r\n");
        PRINT("stack start\t:\t0x%0.8x\r\n", pminfo->stackStart);
        PRINT("stack size\t:\t0x%0.8x\r\n", pminfo->stackSize);
        PRINT("reset reason\t:\t0x%0.8x\r\n", pminfo->resetReason);
        PRINT("stack CRC\t:\t0x%0.8x\r\n", pminfo->stackCrc);
        PRINT("CRC\t\t:\t0x%0.8x\r\n", pminfo->crc);
        PRINT("\r\n");
    }
    else
    {
        PRINT("Structure in not populated!\n");

        return PM_PARSE_FAILED;
    }

    return PM_PARSE_SUCCESS;
}

/**
* @brief Function to print the Post Mortem info
*/
int8_t print_post_mortem_data(struct bhy2_post_mortem *pminfo)
{
    if (pminfo != NULL)
    {
        get_post_mortem_status(pminfo);
        get_post_mortem_context(pminfo);
        get_post_mortem_system_status(pminfo);
        get_post_mortem_diagnostic(pminfo);
        get_post_mortem_stack_info(pminfo);
    }
    else
    {
        PRINT("Structure in not populated!\n");

        return PM_PARSE_FAILED;
    }

    return PM_PARSE_SUCCESS;

}

#endif
