/**
 * @file main.c
 * @author Daan Pape (daan@dptechnics.com)
 * @author Arnoud Devoogdt (arnoud@dptechnics.com)
 * @version 0.2.0
 * @date 2026-04-08
 * @copyright Copyright (c) 2026 DPTechnics BV (info@dptechnics.com)
 * @brief This code connects to the BlueCherry platform using the walter-uc library.
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
 *
 * @section DESCRIPTION
 *
 * This example demonstrates event-driven BlueCherry session management with
 * walter-uc.
 *
 * UC_EVENT_NETWORK_UP triggers a (re)connection to the BlueCherry cloud.
 * UC_EVENT_NETWORK_DOWN marks the session as inactive so a reconnect is
 * attempted once the next UP event is received.
 *
 * Notification bit assignments (FreeRTOS task notifications):
 *   Bit 0 (NET_UP_BIT)   – network up, IP assigned, BlueCherry may connect.
 *   Bit 1 (NET_DOWN_BIT) – network down, BlueCherry session invalidated.
 */

#include <esp_chip_info.h>
#include <bsp/walter.hpp>
#include <esp_system.h>
#include "bluecherry.h"
#include <sdkconfig.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <inttypes.h>
#include <esp_log.h>
#include <string.h>
#include <stdio.h>

#define BLUECHERRY_DEVICE_TYPE "walter01"

/** Logging tag for this module. */
static const char* TAG = "UC_BC";

/** Bit set when a network interface comes up and is ready for use. */
#define NET_UP_BIT   (1UL << 0)

/** Bit set when the active network interface has lost its IP address. */
#define NET_DOWN_BIT (1UL << 1)

/** Handle used by event callbacks to wake the main task. */
static TaskHandle_t s_main_task = nullptr;

/** True while a valid BlueCherry session is believed to be active. */
static bool bc_connected = false;

/**
 * @brief The device certificate from the symbol section of the firmware.
 */
extern const char devcert[] asm("_binary_devcert_pem_start");

/**
 * @brief The device key from the symbol section of the firmware.
 */
extern const char devkey[] asm("_binary_devkey_pem_start");

/**
 * @brief Handle an incoming BlueCherry MQTT message.
 *
 * @param topic The topic index.
 * @param len   The payload length in bytes.
 * @param data  The payload buffer (not null-terminated).
 * @param args  User argument (unused).
 */
static void bluecherry_msg_handler(uint8_t topic, uint16_t len, const uint8_t* data, void* args)
{
  ESP_LOGI(TAG, "Received MQTT message of length %d on topic %02X: %.*s",
           len, topic, len, data);
}

/** Write a string value to the NVS @c bcztp_store namespace. */
static esp_err_t nvs_write_str(const char* key, const char* value)
{
  nvs_handle_t handle;
  ESP_ERROR_CHECK(nvs_open("bcztp_store", NVS_READWRITE, &handle));
  ESP_ERROR_CHECK(nvs_set_str(handle, key, value));
  ESP_ERROR_CHECK(nvs_commit(handle));
  nvs_close(handle);
  return ESP_OK;
}

/** Read a string value from the NVS @c bcztp_store namespace. */
static esp_err_t nvs_read_str(const char* key, char* buf, size_t len)
{
  nvs_handle_t handle;
  esp_err_t    err = nvs_open("bcztp_store", NVS_READONLY, &handle);
  if(err != ESP_OK)
    return err;

  err = nvs_get_str(handle, key, buf, &len);
  nvs_close(handle);
  return err;
}

/**
 * @brief BlueCherry ZTP bio handler – reads and writes certs from/to NVS.
 *
 * @param read   True to read an existing credential, false to write/erase.
 * @param secure True for the private key, false for the certificate.
 * @param args   On write: pointer to the PEM string to store; NULL to erase.
 * @return Pointer to the credential string (read), the written string, or NULL.
 */
static const char* bluecherry_ztp_bio_handler(bool read, bool secure, void* args)
{
  static char devcert[4096];
  static char devkey[4096];

  const char* keyname = secure ? "bcztp_key" : "bcztp_cert";

  if(read) {
    esp_err_t err = nvs_read_str(keyname,
                                 secure ? devkey   : devcert,
                                 secure ? sizeof(devkey) : sizeof(devcert));
    if(err != ESP_OK) {
      ESP_LOGW(TAG, "No %s found in NVS (err=0x%x)", keyname, err);
      return NULL;
    }
    return secure ? devkey : devcert;
  } else {
    const char* data = (const char*) args;

    nvs_handle_t handle;
    ESP_ERROR_CHECK(nvs_open("bcztp_store", NVS_READWRITE, &handle));

    if(data == NULL) {
      ESP_LOGW(TAG, "Erasing %s from NVS", keyname);
      nvs_erase_key(handle, keyname);
      nvs_commit(handle);
      nvs_close(handle);
      return NULL;
    }

    ESP_ERROR_CHECK(nvs_set_str(handle, keyname, data));
    ESP_ERROR_CHECK(nvs_commit(handle));
    nvs_close(handle);

    ESP_LOGI(TAG, "Stored %s in NVS", keyname);
    return data;
  }
}

/**
 * @brief UC network event handler – runs in the default event-loop task.
 *
 * On UC_EVENT_NETWORK_UP:   notify the main task so it can (re)connect to
 *                           BlueCherry on the newly available interface.
 * On UC_EVENT_NETWORK_DOWN: mark the BlueCherry session as invalid and
 *                           notify the main task.
 */
static void uc_network_event_handler(void* arg, esp_event_base_t base,
                                     int32_t event_id, void* event_data)
{
  uc_network_event_t* e = static_cast<uc_network_event_t*>(event_data);

  if(event_id == UC_EVENT_NETWORK_UP) {
    ESP_LOGI(TAG, "Network UP  – driver: %s, IP: " IPSTR,
             e->driver_name, IP2STR(&e->ip_info.ip));

    if(s_main_task) {
      xTaskNotify(s_main_task, NET_UP_BIT, eSetBits);
    }
  } else if(event_id == UC_EVENT_NETWORK_DOWN) {
    ESP_LOGW(TAG, "Network DOWN – driver: %s – BlueCherry session invalidated",
             e->driver_name);

    /* Mark the session as gone; the main task will reconnect on next UP. */
    bc_connected = false;

    if(s_main_task) {
      xTaskNotify(s_main_task, NET_DOWN_BIT, eSetBits);
    }
  }
}

/**
 * @brief The main application entrypoint.
 */
extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "Unified Comms BlueCherry example V0.2.0");

  /* Store the task handle before registering handlers so callbacks can
   * safely notify it as soon as start() makes the first connection. */
  s_main_task = xTaskGetCurrentTaskHandle();

  CELL_DRV(uc.GM02S)->config("CELL-APN", 6);
  WIFI_DRV(uc.ESP_WIFI)->configStation("WIFI-SSID", "WIFI-PASSWORD", 5);

  /* Register UC network event handlers before calling start() so that the
   * very first UC_EVENT_NETWORK_UP event is not missed. */
  uc.controller.registerNetworkEventHandler(UC_EVENT_NETWORK_UP,   uc_network_event_handler, nullptr);
  uc.controller.registerNetworkEventHandler(UC_EVENT_NETWORK_DOWN, uc_network_event_handler, nullptr);

  if(!uc.controller.start()) {
    ESP_LOGE(TAG, "Could not start unified comms – halting");
    return;
  }

  while(true) {
    /* ------------------------------------------------------------------ *
     * Wait for a network state notification from the UC event handler.   *
     * portMAX_DELAY keeps this task suspended until something changes.   *
     * ------------------------------------------------------------------ */
    uint32_t notif = 0;
    xTaskNotifyWait(0, ULONG_MAX, &notif, portMAX_DELAY);

    if(notif & NET_DOWN_BIT) {
      /* Session was already marked invalid in the event handler; nothing
       * further to tear down for BlueCherry (it uses MQTT internally). */
      ESP_LOGW(TAG, "Waiting for network recovery before reconnecting BlueCherry");
    }

    if(notif & NET_UP_BIT) {
      ESP_LOGI(TAG, "Network available – connecting to BlueCherry");

      /* Attempt BlueCherry (re)connection with zero-touch provisioning.
       * Loop until the SDK reports success; each iteration yields the CPU
       * to allow the RTOS scheduler to service other tasks. */

      /* Uncomment to use pre-provisioned keys instead of ZTP:
       * while (!bluecherry_init(devcert, devkey, bluecherry_msg_handler,
       *                         NULL, true, 30)) {
       *   ESP_LOGI(TAG, "Waiting for BlueCherry connection...");
       *   vTaskDelay(pdMS_TO_TICKS(5000));
       * }
       */

      while(bluecherry_init_ztp(bluecherry_ztp_bio_handler, NULL, BLUECHERRY_DEVICE_TYPE,
                                bluecherry_msg_handler, NULL, true, 30) != ESP_OK) {
        ESP_LOGI(TAG, "Waiting for BlueCherry connection...");
        vTaskDelay(pdMS_TO_TICKS(5000));
      }

      bc_connected = true;
      ESP_LOGI(TAG, "BlueCherry connected");
    }

    /* Publish a periodic message while the BlueCherry session is active. */
    if(bc_connected) {
      ESP_LOGI(TAG, "Publishing message");
      bluecherry_publish(0x84, strlen("Test message") + 1,
                         (const uint8_t*) "Test message");

      if(esp_task_wdt_status(NULL) == ESP_OK) {
        esp_task_wdt_reset();
      }

      /* Re-arm the notification wait with a 5-second timeout so we keep
       * publishing while connected but still react promptly to DOWN events. */
      xTaskNotifyWait(0, ULONG_MAX, &notif, pdMS_TO_TICKS(5000));
      if(notif & NET_DOWN_BIT) {
        bc_connected = false;
        ESP_LOGW(TAG, "Network lost during publish – waiting for recovery");
      }
    }
  }
}