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
 * @file    cli.c
 * @brief   Source file for the command line utility
 *
 */

#include "cli.h"

#include <string.h>
#include <stdbool.h>
#include <stdio.h>

int8_t cli_run(uint8_t argc, uint8_t *argv[], const cli_dev_t *dev)
{
    int16_t arg_i, tab_i;
    int8_t ret = CLI_OK;
    bool cmd_processed = false;
    bool invalid_cmd;

    if (!dev || !argv)
    {
        return CLI_E_NULL_PTR;
    }

    /* Loop 1 runs through the list of arguments.
     * Loop 2 runs through the look up table of available commands
     * If a command matches, it checks if there are sufficient arguments.
     * If there are insufficient arguments, then prompt the commands specific help.
     * If there are sufficient arguments, then execute the callback with the arguments.
     * Increment the argument list for Loop 1 by exiting Loop 2.
     */

    for (arg_i = 0; (arg_i < argc) && (ret == CLI_OK);)
    {
        for (tab_i = 0; (tab_i < dev->n_cmds) && (ret == CLI_OK);)
        {
            uint8_t cmd_char_str[5] = { 0 };
            sprintf((char *)cmd_char_str, "-%c", dev->table[tab_i].cmd_char);
            invalid_cmd = strcmp((const char *)argv[arg_i], (const char *)dev->table[tab_i].cmd_str) != 0;
            if (invalid_cmd)
            {
                if ((dev->table[tab_i].cmd_char != 0) &&
                    (strcmp((const char *)argv[arg_i], (const char *)cmd_char_str) == 0))
                {
                    invalid_cmd = false;
                }
            }

            if (invalid_cmd)
            {
                tab_i++;
                continue;
            }
            else
            {
                if ((argc - arg_i) < (dev->table[tab_i].n_args + 1))
                {
                    /* If arguments are required and not enough
                     * arguments are available, print help */

                    cmd_processed = true;

                    /* Avoid Null pointer calls */
                    if (dev->table[tab_i].help_callback)
                    {
                        return dev->table[tab_i].help_callback(dev->ref);
                    }
                }
                else
                {
                    /* If enough arguments are available */

                    cmd_processed = true;

                    /* Avoid Null pointer calls */
                    if (dev->table[tab_i].callback)
                    {
                        ret = dev->table[tab_i].callback(dev->table[tab_i].n_args, &argv[arg_i], dev->ref);

                        if (ret == CLI_OK)
                        {
                            arg_i += dev->table[tab_i].n_args;
                        }
                        else
                        {
                            return ret;
                        }
                    }
                }

                tab_i = dev->n_cmds;
            }
        }

        arg_i++;
    }

    /* Returns CLI_OK if at least one of the commands was valid */
    if (cmd_processed)
    {
        return CLI_OK;
    }
    else
    {
        return CLI_E_UNKNOWN_CMD;
    }
}

int8_t cli_help(void *ref, const cli_dev_t *dev)
{
    uint8_t cmd_i;
    int8_t ret = CLI_OK;

    for (cmd_i = 0; cmd_i < dev->n_cmds; cmd_i++)
    {
        /* Avoid Null pointer calls */
        if (dev->table[cmd_i].help_callback)
        {
            ret = dev->table[cmd_i].help_callback(ref);

            if (ret != CLI_OK)
            {
                break;
            }
        }
    }

    return ret;
}
