/**
 * @file main.c
 * @author Daan Pape (daan@dptechnics.com)
 * @author Arnoud Devoogdt (arnoud@dptechnics.com)
 * @version 0.2.0
 * @date 2026-04-08
 * @copyright Copyright (c) 2026 DPTechnics BV (info@dptechnics.com)
 * @brief This code connects to a UDP socket using the walter-uc library.
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
 * This example demonstrates event-driven network management with walter-uc.
 *
 * The UC_EVENT_NETWORK_UP / UC_EVENT_NETWORK_DOWN callbacks notify the main
 * task via FreeRTOS task notifications whenever the active network interface
 * changes.  The main task uses these signals to open or close the UDP socket
 * instead of relying on blind retry loops.
 *
 * Notification bit assignments:
 *   Bit 0 (NET_UP_BIT)   – a network interface is up and an IP was assigned.
 *   Bit 1 (NET_DOWN_BIT) – the active network interface lost its IP address.
 */

#include <string.h>
#include <stdio.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <bsp/walter.hpp>
#include <esp_mac.h>

static const char* TAG = "UC_UDP";

/** Bit set in the main task's notification value when a network comes up. */
#define NET_UP_BIT   (1UL << 0)

/** Bit set in the main task's notification value when a network goes down. */
#define NET_DOWN_BIT (1UL << 1)

/** Handle used by UC event callbacks to wake the main task. */
static TaskHandle_t s_main_task = nullptr;

/** Most-recently received IP information, written by the UC_EVENT_NETWORK_UP
 *  callback and read by the main task.  Access is safe because the main task
 *  only reads the struct after receiving NET_UP_BIT. */
static uc_network_event_t s_net_info = {};

/**
 * @brief UC network event handler – runs in the default event-loop task.
 *
 * On UC_EVENT_NETWORK_UP:   store the IP info and notify the main task so
 *                           it can (re)open the UDP socket.
 * On UC_EVENT_NETWORK_DOWN: notify the main task so it closes the socket
 *                           before the underlying interface disappears.
 */
static void uc_network_event_handler(void* arg, esp_event_base_t base,
                                     int32_t event_id, void* event_data)
{
  uc_network_event_t* e = static_cast<uc_network_event_t*>(event_data);

  if(event_id == UC_EVENT_NETWORK_UP) {
    ESP_LOGI(TAG, "Network UP  – driver: %s, IP: " IPSTR,
             e->driver_name, IP2STR(&e->ip_info.ip));

    /* Snapshot the network info for the main task. */
    s_net_info = *e;

    if(s_main_task) {
      xTaskNotify(s_main_task, NET_UP_BIT, eSetBits);
    }
  } else if(event_id == UC_EVENT_NETWORK_DOWN) {
    ESP_LOGW(TAG, "Network DOWN – driver: %s – closing socket", e->driver_name);

    if(s_main_task) {
      xTaskNotify(s_main_task, NET_DOWN_BIT, eSetBits);
    }
  }
}

/**
 * @brief Open a UDP socket to @p host : @p port.
 *
 * Blocks until DNS resolution and socket creation succeed, then returns the
 * socket file descriptor.  The caller is responsible for calling
 * freeaddrinfo(*out_res) when the socket is no longer needed.
 *
 * @param host     Hostname or dotted-decimal IP string.
 * @param port     Service name or decimal port number string.
 * @param out_res  Receives the addrinfo chain; caller calls freeaddrinfo.
 * @return         A valid socket file descriptor (≥ 0).
 */
static int open_udp_socket(const char* host, const char* port, struct addrinfo** out_res)
{
  struct addrinfo hints = {};
  hints.ai_family   = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;

  while(true) {
    int err = getaddrinfo(host, port, &hints, out_res);
    if(err != 0 || *out_res == nullptr) {
      ESP_LOGE(TAG, "DNS lookup failed (%d), retrying in 5 s…", err);
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    int sock = socket((*out_res)->ai_family, (*out_res)->ai_socktype, 0);
    if(sock < 0) {
      ESP_LOGE(TAG, "socket() failed, retrying in 5 s…");
      freeaddrinfo(*out_res);
      *out_res = nullptr;
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    ESP_LOGI(TAG, "UDP socket opened (fd=%d)", sock);
    return sock;
  }
}

extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "Unified Comms UDP Socket example V0.2.0");

  /* Store the task handle before registering event handlers so the callbacks
   * can safely notify it the moment start() triggers the first connection. */
  s_main_task = xTaskGetCurrentTaskHandle();

  CELL_DRV(uc.GM02S)->config("CELL-APN", 6, 600000);    // 600 s timeout for cellular
  WIFI_DRV(uc.ESP_WIFI)->configStation("WIFI-SSID", "WIFI-PASSWORD", 5, 20000); // 20 s timeout for Wi-Fi

  /* Register UC network event handlers before calling start() so that the
   * very first UC_EVENT_NETWORK_UP event is not missed. */
  uc.controller.registerNetworkEventHandler(UC_EVENT_NETWORK_UP,   uc_network_event_handler, nullptr);
  uc.controller.registerNetworkEventHandler(UC_EVENT_NETWORK_DOWN, uc_network_event_handler, nullptr);

  if(!uc.controller.start()) {
    ESP_LOGE(TAG, "Could not start unified comms – halting");
    return;
  }

  const char* host = "walterdemo.quickspot.io";
  const char* port = "1999";

  uint8_t  mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);

  struct addrinfo* res           = nullptr;
  int              sock          = -1;
  uint16_t         counter       = 0;
  bool             network_ready = false;

  while(true) {
    /* ------------------------------------------------------------------ *
     * Block until notified by the UC event handler, or wake every 2 s    *
     * so send failures also result in a timely reconnect attempt.        *
     * ------------------------------------------------------------------ */
    uint32_t notif = 0;
    xTaskNotifyWait(0, ULONG_MAX, &notif, pdMS_TO_TICKS(2000));

    /* Process DOWN before UP so a rapid DOWN → UP pair always results in
     * a full, clean socket rebuild. */
    if(notif & NET_DOWN_BIT) {
      network_ready = false;
      if(sock >= 0) {
        ESP_LOGW(TAG, "Closing UDP socket due to network loss");
        close(sock);
        sock = -1;
      }
      if(res) {
        freeaddrinfo(res);
        res = nullptr;
      }
    }

    if(notif & NET_UP_BIT) {
      /* A (possibly new) interface is ready – rebuild the socket. */
      if(sock >= 0) {
        close(sock);
        sock = -1;
      }
      if(res) {
        freeaddrinfo(res);
        res = nullptr;
      }
      ESP_LOGI(TAG, "Opening UDP socket to %s:%s", host, port);
      sock          = open_udp_socket(host, port, &res);
      network_ready = true;
    }

    /* Nothing to do until we have a valid socket. */
    if(!network_ready || sock < 0) {
      continue;
    }

    /* Build and send an 8-byte probe packet: 6-byte MAC + 2-byte counter. */
    uint8_t buffer[8];
    memcpy(buffer, mac, 6);
    buffer[6] = (counter >> 8) & 0xFF;
    buffer[7] =  counter       & 0xFF;
    counter++;

    int sent = sendto(sock, buffer, sizeof(buffer), 0,
                      res->ai_addr, res->ai_addrlen);
    if(sent < 0) {
      /* The send failed – most likely the interface went away without a
       * clean IP-lost event.  Drop the socket and wait for the UC
       * controller to emit UC_EVENT_NETWORK_UP once it reconnects. */
      ESP_LOGE(TAG, "sendto() failed – waiting for network recovery");
      close(sock);
      sock = -1;
      freeaddrinfo(res);
      res           = nullptr;
      network_ready = false;
    } else {
      ESP_LOGI(TAG, "Sent UDP packet #%" PRIu16, counter);
    }
  }
}