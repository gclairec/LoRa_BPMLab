/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "freertos/FreeRTOS.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void gatts_main(void);
uint8_t data_avail_chk(void);
void clear_avail_flag(void);
uint8_t *getData_to_send(void);

/*
 * Data status
 */
#define DATA_AVAIL_NO                   0
#define DATA_AVAIL_YES					1


/* Attributes State Machine */
enum
{
    IDX_SVC,
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,

    IDX_CHAR_B,
    IDX_CHAR_VAL_B,

    IDX_CHAR_C,
    IDX_CHAR_VAL_C,

    HRS_IDX_NB,
};

