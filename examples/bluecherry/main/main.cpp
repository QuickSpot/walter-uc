/**
 * @file main.c
 * @author Daan Pape (daan@dptechnics.com)
 * @brief This code connects to the BlueCherry platform.
 * @version 0.1
 * @date 2025-07-14
 * @copyright Copyright (c) 2025 DPTechnics BV
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/lgpl-3.0.html>.
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <esp_log.h>
#include <sdkconfig.h>
#include <esp_chip_info.h>
#include <esp_system.h>
#include <esp_event.h>
#include "bluecherry.h"
#include <bsp/walter.hpp>

/**
 * @brief The logging tag for this BlueCherry module.
 */
static const char* TAG = "UC_BC";

/**
 * @brief The device certificate from the symbol section of the firmware.
 */
extern const char devcert[] asm("_binary_devcert_pem_start");

/**
 * @brief The device key from the symbol section of the firmware.
 */
extern const char devkey[] asm("_binary_devkey_pem_start");

/**
 * @brief Handle an incoming MQTT message.
 *
 * This function handles an incoming MQTT message.
 *
 * @param topic The topic as the topic index.
 * @param len The length of the incoming data.
 * @param data The incoming data buffer.
 * @param args A NULL pointer.
 */
static void bluecherry_msg_handler(uint8_t topic, uint16_t len, const uint8_t* data, void* args)
{
  ESP_LOGI(TAG, "Received MQTT message of length %d on topic %02X: %.*s", len, topic, len, data);
}

/**
 * @brief The main application entrypoint.
 */
extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "Unified Comms BlueCherry example V1.0.0");

  CELL_DRV(uc.GM02S)->config("CELL-APN", 6);
  WIFI_DRV(uc.ESP_WIFI)->configStation("WIFI-SSID", "WIFI-PASSWORD", 5);

  if(uc.controller.start()) {
    ESP_LOGI(TAG, "Succesfully started unified comms");
  } else {
    ESP_LOGE(TAG, "Could not start unified comms / connect a driver");
  }

  vTaskDelay(pdMS_TO_TICKS(5000));

  while(true) {
    if(bluecherry_init(devcert, devkey, bluecherry_msg_handler, NULL, true) == ESP_OK)
      break;
    vTaskDelay(pdMS_TO_TICKS(10000));
  }

  while(true) {
    ESP_LOGI(TAG, "Publishing test message");
    bluecherry_publish(0x84, strlen("Test message") + 1, (const uint8_t*) "Test message");
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}