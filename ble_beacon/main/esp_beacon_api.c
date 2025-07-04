/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */



/****************************************************************************
*
* This file is for iBeacon APIs. It supports both iBeacon encode and decode.
*
* iBeacon is a trademark of Apple Inc. Before building devices which use iBeacon technology,
* visit https://developer.apple.com/ibeacon/ to obtain a license.
*
****************************************************************************/
#include "esp_beacon_api.h"

const uint8_t uuid_zeros[ESP_UUID_LEN_128] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t known_mac_addr_1[6] = {0x5B, 0x1E, 0xED, 0xE6, 0xE9, 0x83};
const uint8_t known_mac_addr_2[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const uint8_t known_mac_addr_3[6] = {0x5B, 0x58, 0x34, 0xF2, 0x7F, 0x7E};
const uint8_t known_mac_addr_4[6] = {0x5B, 0x1A, 0x00, 0xDD, 0xF9, 0x13};

int8_t measure_first[NUM_ITERATIONS] = {0};
int8_t measure_second[NUM_ITERATIONS] = {0};
int8_t measure_third[NUM_ITERATIONS] = {0};
int8_t measure_forth[NUM_ITERATIONS] = {0};

uint8_t num_first = 0, num_second = 0, num_third = 0, num_forth = 0;
bool stop_read_first = false, stop_read_second = false, stop_read_third = false, stop_read_forth = false;


bool esp_ble_is_known_device (uint8_t *mac){
    bool result = false;

    if (memcmp(mac, known_mac_addr_1, sizeof(known_mac_addr_1)) == 0 || memcmp(mac, known_mac_addr_2, sizeof(known_mac_addr_2)) == 0 || memcmp(mac, known_mac_addr_3, sizeof(known_mac_addr_3)) == 0 || memcmp(mac, known_mac_addr_4, sizeof(known_mac_addr_4)) == 0) {
            result = true;
        }

    return result;
}

void print_rssi_values(uint8_t num, int8_t *measure) {
    for (int i = 0; i < num; i++) {
        if (i > 0) {
            printf(", ");
        }
        printf("%d", measure[i]);
    }
    printf("\n");   
}
