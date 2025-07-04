/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


/****************************************************************************
*
* This file is for iBeacon definitions. It supports both iBeacon sender and receiver
* which is distinguished by macros IBEACON_SENDER and IBEACON_RECEIVER,
*
* iBeacon is a trademark of Apple Inc. Before building devices which use iBeacon technology,
* visit https://developer.apple.com/ibeacon/ to obtain a license.
*
****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"

#define IBEACON_SENDER      0
#define IBEACON_RECEIVER    1
#define IBEACON_MODE CONFIG_IBEACON_MODE

/* Major and Minor part are stored in big endian mode in iBeacon packet,
 * need to use this macro to transfer while creating or processing
 * iBeacon data */

#define ENDIAN_CHANGE_U16(x) ((((x)&0xFF00)>>8) + (((x)&0xFF)<<8))

/* Espressif WeChat official account can be found using WeChat "Yao Yi Yao Zhou Bian",
 * if device advertises using ESP defined UUID.
 * Please refer to http://zb.weixin.qq.com for further information. */

#define ESP_UUID    {0xFD, 0xA5, 0x06, 0x93, 0xA4, 0xE2, 0x4F, 0xB1, 0xAF, 0xCF, 0xC6, 0xEB, 0x07, 0x64, 0x78, 0x25}
#define ESP_MAJOR   10167
#define ESP_MINOR   61958
#define NUM_ITERATIONS  10 

extern const uint8_t known_mac_addr_1[6];
extern const uint8_t known_mac_addr_2[6];
extern const uint8_t known_mac_addr_3[6];
extern const uint8_t known_mac_addr_4[6];

extern int8_t measure_first[NUM_ITERATIONS];
extern int8_t measure_second[NUM_ITERATIONS];
extern int8_t measure_third[NUM_ITERATIONS];
extern int8_t measure_forth[NUM_ITERATIONS];

extern uint8_t num_first, num_second, num_third, num_forth;
extern bool stop_read_first, stop_read_second, stop_read_third, stop_read_forth;

bool esp_ble_is_known_device(uint8_t *mac);
void print_rssi_values(uint8_t num, int8_t *measure);

