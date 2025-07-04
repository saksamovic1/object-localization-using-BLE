/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */



/****************************************************************************
*
* This file is for iBeacon demo. It supports both iBeacon sender and receiver
* which is distinguished by macros IBEACON_SENDER and IBEACON_RECEIVER,
*
* iBeacon is a trademark of Apple Inc. Before building devices which use iBeacon technology,
* visit https://developer.apple.com/ibeacon/ to obtain a license.
*
****************************************************************************/
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_beacon_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define BUTTON_GPIO 25

static const char* DEMO_TAG = "IBEACON_DEMO";
TaskHandle_t taskHandle = NULL;
volatile BaseType_t xHigherPriorityTaskWoken = pdFALSE;
volatile bool button_pressed = false;

///Declare static functions
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

#if (IBEACON_MODE == IBEACON_RECEIVER)
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};
#endif

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;

    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
#if (IBEACON_MODE == IBEACON_RECEIVER)
        //the unit of the duration is second, 0 means scan permanently
        uint32_t duration = 0;
        esp_ble_gap_start_scanning(duration);
#endif
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(DEMO_TAG, "Scanning start failed, error %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(DEMO_TAG, "Scanning start successfully");
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //adv start complete event to indicate adv start successfully or failed
        if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(DEMO_TAG, "Advertising start failed, error %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(DEMO_TAG, "Advertising start successfully");
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            /* Search for BLE beacon packet */
            if (esp_ble_is_known_device(scan_result->scan_rst.bda)) {
                if (!stop_read_first && memcmp(scan_result->scan_rst.bda, known_mac_addr_1, sizeof(known_mac_addr_1)) == 0){
                    ESP_LOGI("BLE_DEVICE", "Found known device with MAC address: "ESP_BD_ADDR_STR"", ESP_BD_ADDR_HEX(scan_result->scan_rst.bda));
                    ESP_LOGI("BLE_DEVICE", "RSSI: %d dBm", scan_result->scan_rst.rssi);
                    measure_first[num_first] = scan_result->scan_rst.rssi;
                    num_first++;
                    
                    if (num_first == NUM_ITERATIONS) {
                        stop_read_first = true;
                        ESP_LOGI("BLE_DEVICE", "Finished measuring for first beacon - 5b:1e:ed:e6:e9:83!");
                        ESP_LOGI("BLE_DEVICE", "Values in measure_first array:");
                        print_rssi_values(num_first, &measure_first[0]);
                    }

                }
                if (!stop_read_second && memcmp(scan_result->scan_rst.bda, known_mac_addr_2, sizeof(known_mac_addr_2)) == 0) {
                    ESP_LOGI("BLE_DEVICE", "Found known device with MAC address: "ESP_BD_ADDR_STR"", ESP_BD_ADDR_HEX(scan_result->scan_rst.bda));
                    ESP_LOGI("BLE_DEVICE", "RSSI: %d dBm", scan_result->scan_rst.rssi);
                    measure_second[num_second] = scan_result->scan_rst.rssi;
                    num_second++;

                    if (num_second == NUM_ITERATIONS) {
                        stop_read_second = true;
                        ESP_LOGI("BLE_DEVICE", "Finished measuring for second beacon - ff:ff:ff:ff:ff:ff!");
                        ESP_LOGI("BLE_DEVICE", "Values in measure_second array:");
                        print_rssi_values(num_second, &measure_second[0]);
                    }
                }

                if (!stop_read_third && memcmp(scan_result->scan_rst.bda, known_mac_addr_3, sizeof(known_mac_addr_3)) == 0) {
                    ESP_LOGI("BLE_DEVICE", "Found known device with MAC address: "ESP_BD_ADDR_STR"", ESP_BD_ADDR_HEX(scan_result->scan_rst.bda));
                    ESP_LOGI("BLE_DEVICE", "RSSI: %d dBm", scan_result->scan_rst.rssi);
                    measure_third[num_third] = scan_result->scan_rst.rssi;
                    num_third++;

                    if (num_third == NUM_ITERATIONS) {
                        stop_read_third = true;
                        ESP_LOGI("BLE_DEVICE", "Finished measuring for third beacon - 5b:58:34:f2:7f:7e!");
                        ESP_LOGI("BLE_DEVICE", "Values in measure_third array:");
                        print_rssi_values(num_third, &measure_third[0]);
                    }
                }

                if (!stop_read_forth && memcmp(scan_result->scan_rst.bda, known_mac_addr_4, sizeof(known_mac_addr_4)) == 0) {
                    ESP_LOGI("BLE_DEVICE", "Found known device with MAC address: "ESP_BD_ADDR_STR"", ESP_BD_ADDR_HEX(scan_result->scan_rst.bda));
                    ESP_LOGI("BLE_DEVICE", "RSSI: %d dBm", scan_result->scan_rst.rssi);
                    measure_forth[num_forth] = scan_result->scan_rst.rssi;
                    num_forth++;

                    if (num_forth == NUM_ITERATIONS) {
                        stop_read_forth = true;
                        ESP_LOGI("BLE_DEVICE", "Finished measuring for forth beacon - 5b:1a:00:dd:f9:13!");
                        ESP_LOGI("BLE_DEVICE", "Values in measure_forth array:");
                        print_rssi_values(num_forth, &measure_forth[0]);
                    }
                }
            }
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(DEMO_TAG, "Scanning stop failed, error %s", esp_err_to_name(err));
        }
        else {
            ESP_LOGI(DEMO_TAG, "Scanning stop successfully");
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(DEMO_TAG, "Advertising stop failed, error %s", esp_err_to_name(err));
        }
        else {
            ESP_LOGI(DEMO_TAG, "Advertising stop successfully");
        }
        break;

    default:
        break;
    }
}

void button_task(void *pvParameters) {
    while (1) {
        if (button_pressed) {
            ESP_LOGI("BUTTON_TASK", "Button pressed!");

            // Resetirajte flag i obradite događaj
            button_pressed = false;
            stop_read_first = false;
            stop_read_second = false;
            stop_read_third = false;
            stop_read_forth = false;
            num_first = 0;
            num_second = 0;
            num_third = 0;
            num_forth = 0;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // Kratki delay za rasterećenje CPU-a
    }
}

// Funkcija za konfiguraciju GPIO pina
void gpio_isr_handler(void* arg) {
    button_pressed = true; // Set flag when button is pressed
    // Možete koristiti xTaskNotifyFromISR za signaliziranje zadatka
    xTaskNotifyFromISR(taskHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
}

void gpio_init() {
    esp_rom_gpio_pad_select_gpio(BUTTON_GPIO);  // Odabir pina
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);  // Postavljanje pina kao ulaz
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);  // Aktiviranje pull-up otpornika

    // Konfiguracija prekida na padajući rub
    gpio_set_intr_type(BUTTON_GPIO, GPIO_INTR_NEGEDGE);
    
    // Instalacija GPIO ISR (Interrupt Service Routine)
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    gpio_isr_handler_add(BUTTON_GPIO, gpio_isr_handler, NULL);
}


void ble_beacon_appRegister(void)
{
    esp_err_t status;

    ESP_LOGI(DEMO_TAG, "register callback");

    //register the scan callback function to the gap module
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(DEMO_TAG, "gap register error: %s", esp_err_to_name(status));
        return;
    }

}

void ble_beacon_init(void)
{
    esp_bluedroid_init();
    esp_bluedroid_enable();
    ble_beacon_appRegister();
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    ble_beacon_init();
    gpio_init();

    xTaskCreate(button_task, "Button Task", 2048, NULL, 10, &taskHandle);
    /* set scan parameters */
#if (IBEACON_MODE == IBEACON_RECEIVER)
    esp_ble_gap_set_scan_params(&ble_scan_params);
#endif
}