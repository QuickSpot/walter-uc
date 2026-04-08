/**
 * @file main.c
 * @author Daan Pape (daan@dptechnics.com)
 * @author Arnoud Devoogdt (arnoud@dptechnics.com)
 * @version 0.2.0
 * @date 2026-04-08
 * @copyright Copyright (c) 2026 DPTechnics BV (info@dptechnics.com)
 * @brief This code connects to an MQTT platform using the walter-uc library.
 *
 * @section LICENSE
 *
 * Copyright (C) 2026, DPTechnics bv
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *   3. Neither the name of DPTechnics bv nor the names of its contributors may
 *      be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 *   4. This software, with or without modification, must only be used with a
 *      Walter board from DPTechnics bv.
 *
 *   5. Any software provided in binary form under this license must not be
 *      reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY DPTECHNICS BV “AS IS” AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DPTECHNICS BV OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 *
 * This example demonstrates event-driven MQTT session management with walter-uc.
 *
 * The UC_EVENT_NETWORK_UP callback starts (or restarts) the MQTT client once an
 * IP address has been assigned.  UC_EVENT_NETWORK_DOWN stops the MQTT client so
 * the session is torn down cleanly before the underlying interface changes.
 *
 * Notification bit assignments (FreeRTOS task notifications):
 *   Bit 0 (NET_UP_BIT)   – network interface up, IP assigned, MQTT may start.
 *   Bit 1 (NET_DOWN_BIT) – network interface down, stop MQTT client.
 */

#include "mqtt_client.h"
#include <bsp/walter.hpp>
#include <netif/ppp/ppp.h>

static const char* TAG = "UC_MQTT";

/** Bit set when a network interface comes up and is ready for use. */
#define NET_UP_BIT   (1UL << 0)

/** Bit set when the active network interface has lost its IP address. */
#define NET_DOWN_BIT (1UL << 1)

/** Handle used by event callbacks to wake the main task. */
static TaskHandle_t s_main_task = nullptr;

static bool                     mqtt_connected = false;
static esp_mqtt_client_handle_t client         = nullptr;

static void mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id,
                               void* event_data)
{
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;

  switch(event_id) {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT connected");
    {
      const char* topic   = "walter-mqtt-test-topic";
      const char* message = "Hello from Walter device!";
      int         msg_id  = esp_mqtt_client_publish(client, topic, message, 0, 1, 0);
      ESP_LOGI(TAG, "Published message with id: %d", msg_id);
    }
    mqtt_connected = true;
    break;

  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT disconnected");
    mqtt_connected = false;
    break;

  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI(TAG, "MQTT message published, msg_id=%d", event->msg_id);
    break;

  case MQTT_EVENT_ERROR:
    ESP_LOGE(TAG, "MQTT event error");
    break;

  default:
    break;
  }
}

/**
 * @brief UC network event handler – runs in the default event-loop task.
 *
 * On UC_EVENT_NETWORK_UP:   notify the main task so it (re)starts the MQTT
 *                           client on the newly available interface.
 * On UC_EVENT_NETWORK_DOWN: notify the main task so it stops the MQTT
 *                           client cleanly before the interface disappears.
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
    ESP_LOGW(TAG, "Network DOWN – driver: %s – stopping MQTT client", e->driver_name);

    if(s_main_task) {
      xTaskNotify(s_main_task, NET_DOWN_BIT, eSetBits);
    }
  }
}

extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "Unified Comms MQTT example V0.2.0");

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

  /* Create the MQTT client once; it will be started/stopped in response to
   * network events rather than after an arbitrary delay. */
  esp_mqtt_client_config_t mqtt_cfg   = {};
  mqtt_cfg.broker.address.uri         = "mqtt://broker.hivemq.com:1883";
  client                              = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(client, esp_mqtt_event_id_t::MQTT_EVENT_ANY,
                                 mqtt_event_handler, nullptr);

  while(true) {
    /* ------------------------------------------------------------------ *
     * Suspend until the UC event handler sends a notification.           *
     * Using portMAX_DELAY avoids a busy-wait; the task is only woken     *
     * when the network state actually changes.                           *
     * ------------------------------------------------------------------ */
    uint32_t notif = 0;
    xTaskNotifyWait(0, ULONG_MAX, &notif, portMAX_DELAY);

    /* Process DOWN before UP so a rapid DOWN → UP pair results in a clean
     * client restart. */
    if(notif & NET_DOWN_BIT) {
      if(mqtt_connected || client) {
        ESP_LOGI(TAG, "Stopping MQTT client due to network loss");
        esp_mqtt_client_stop(client);
        mqtt_connected = false;
      }
    }

    if(notif & NET_UP_BIT) {
      ESP_LOGI(TAG, "Starting MQTT client");
      esp_mqtt_client_start(client);
    }

    /* Publish a keep-alive message if the MQTT session is active. */
    if(mqtt_connected) {
      const char* topic   = "walter-mqtt-test-topic";
      const char* message = "Hello again from Walter!";
      int         msg_id  = esp_mqtt_client_publish(client, topic, message, 0, 1, 0);
      ESP_LOGI(TAG, "Published in loop, msg_id=%d", msg_id);
    }
  }
}