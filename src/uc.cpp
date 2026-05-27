/**
 * @file uc.cpp
 * @author Daan Pape <daan@dptechnics.com>, Arnoud Devoogdt <arnoud@dptechnics.com>, Robbe Beernaert
 * @date 8 Aug 2025
 * @copyright DPTechnics bv
 * @brief Walter unified comms library
 *
 * @section LICENSE
 *
 * Copyright (C) 2025, DPTechnics bv
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this list of
 *      conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright notice, this list of
 *      conditions and the following disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of DPTechnics bv nor the names of its contributors may be used to endorse
 *      or promote products derived from this software without specific prior written permission.
 *
 *   4. This software, with or without modification, must only be used with a Walter board from
 *      DPTechnics bv.
 *
 *   5. Any software provided in binary form under this license must not be reverse engineered,
 *      decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY DPTECHNICS BV “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT, AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL DPTECHNICS BV OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 *
 * This file contains the unified comms library for Walter.
 */

#include "uc.hpp"
#include <algorithm>
#include <driver/wifiDriver.hpp>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_log.h>
#include <ping/ping_sock.h>
#include <lwip/ip_addr.h>
#include <lwip/inet.h>
#include <freertos/semphr.h>

ESP_EVENT_DEFINE_BASE(UC_NETWORK_EVENT_BASE);

#define PPP_IP_BIT (1 << 0)
#define STA_IP_BIT (1 << 1)
#define ETH_IP_BIT (1 << 2)

EventGroupHandle_t ip_event_group;

EventBits_t interfaceTypeToBit(driver::InterfaceType type)
{
  switch(type) {
  case driver::WIFI:
    return STA_IP_BIT;
  case driver::ETHERNET:
    return ETH_IP_BIT;
  case driver::CELLULAR:
    return PPP_IP_BIT;
  default:
    return 0;
  }
}

namespace {
constexpr const char* INTERNET_PING_TARGET = "8.8.8.8";

struct PingContext {
  SemaphoreHandle_t done;
  bool success;
};

static void _on_ping_success(esp_ping_handle_t hdl, void* args)
{
  (void) hdl;
  PingContext* ctx = static_cast<PingContext*>(args);
  if(ctx != nullptr) {
    ctx->success = true;
  }
}

static void _on_ping_end(esp_ping_handle_t hdl, void* args)
{
  (void) hdl;
  PingContext* ctx = static_cast<PingContext*>(args);
  if(ctx != nullptr && ctx->done != nullptr) {
    xSemaphoreGive(ctx->done);
  }
}

static bool _pingEndpointOnce(uint32_t pingTimeoutMs)
{
  ip_addr_t target_addr = {};
  if(!ipaddr_aton(INTERNET_PING_TARGET, &target_addr)) {
    ESP_LOGE(LOGTAG, "Invalid internet ping target: %s", INTERNET_PING_TARGET);
    return false;
  }

  PingContext ctx = {
    .done = xSemaphoreCreateBinary(),
    .success = false,
  };
  if(ctx.done == nullptr) {
    ESP_LOGE(LOGTAG, "Failed to create ping sync semaphore");
    return false;
  }

  esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
  ping_config.target_addr = target_addr;
  ping_config.count = 1;
  ping_config.interval_ms = pingTimeoutMs;
  ping_config.timeout_ms = pingTimeoutMs;

  esp_ping_callbacks_t callbacks = {
    .cb_args = &ctx,
    .on_ping_success = _on_ping_success,
    .on_ping_timeout = nullptr,
    .on_ping_end = _on_ping_end,
  };

  esp_ping_handle_t ping_handle = nullptr;
  esp_err_t ret = esp_ping_new_session(&ping_config, &callbacks, &ping_handle);
  if(ret != ESP_OK) {
    ESP_LOGW(LOGTAG, "Failed to create ping session: %s", esp_err_to_name(ret));
    vSemaphoreDelete(ctx.done);
    return false;
  }

  ret = esp_ping_start(ping_handle);
  if(ret != ESP_OK) {
    ESP_LOGW(LOGTAG, "Failed to start ping session: %s", esp_err_to_name(ret));
    esp_ping_delete_session(ping_handle);
    vSemaphoreDelete(ctx.done);
    return false;
  }

  const TickType_t waitTicks = pdMS_TO_TICKS(pingTimeoutMs + 1000);
  bool finished = xSemaphoreTake(ctx.done, waitTicks) == pdTRUE;

  esp_ping_stop(ping_handle);
  esp_ping_delete_session(ping_handle);
  vSemaphoreDelete(ctx.done);

  return finished && ctx.success;
}

static bool _performInternetCheckWithinTimeout(uint32_t totalTimeoutSeconds)
{
  const uint32_t totalTimeoutMs = totalTimeoutSeconds * 1000;
  const uint32_t perAttemptMs = 2000;
  const TickType_t startTick = xTaskGetTickCount();

  do {
    if(_pingEndpointOnce(perAttemptMs)) {
      return true;
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    const TickType_t elapsed = xTaskGetTickCount() - startTick;
    if(pdTICKS_TO_MS(elapsed) >= totalTimeoutMs) {
      break;
    }
  } while(true);

  return false;
}

static bool _verifyInternetReachabilityOnDriver(driver::Driver* candidate)
{
  constexpr uint32_t PING_TIMEOUT_S = 10;
  esp_netif_t* previousDefault = esp_netif_get_default_netif();
  esp_netif_set_default_netif(candidate->getInterface());

  ESP_LOGI(LOGTAG, "Running internet check for %.*s …",
           (int) candidate->name.size(), candidate->name.data());

  bool success = _performInternetCheckWithinTimeout(PING_TIMEOUT_S);

  if(success) {
    ESP_LOGI(LOGTAG, "Internet check passed for %.*s",
             (int) candidate->name.size(), candidate->name.data());
  } else {
    ESP_LOGW(LOGTAG, "Internet check failed for %.*s",
             (int) candidate->name.size(), candidate->name.data());
    if(previousDefault != nullptr) {
      esp_netif_set_default_netif(previousDefault);
    }
  }

  return success;
}

static void _postNetworkUpEvent(driver::InterfaceType type, const char* driverName,
                                esp_netif_t* netif)
{
  esp_netif_ip_info_t ip_info = {};
  esp_netif_dns_info_t dns_main = {};
  esp_netif_dns_info_t dns_backup = {};

  if(esp_netif_get_ip_info(netif, &ip_info) != ESP_OK) {
    ESP_LOGW(LOGTAG, "Could not read IP info for %s while posting NETWORK_UP", driverName);
    return;
  }

  esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns_main);
  esp_netif_get_dns_info(netif, ESP_NETIF_DNS_BACKUP, &dns_backup);

  uc_network_event_t net_event = {
    .interface_type = type,
    .driver_name = driverName,
    .ip_info = ip_info,
    .dns_main = dns_main.ip.u_addr.ip4,
    .dns_backup = dns_backup.ip.u_addr.ip4,
  };

  esp_event_post(UC_NETWORK_EVENT_BASE, UC_EVENT_NETWORK_UP,
                 &net_event, sizeof(net_event), portMAX_DELAY);
}

static const char* _driverNameForType(driver::InterfaceType type)
{
  switch(type) {
  case driver::WIFI:
    return "Espressif WiFi";
  case driver::CELLULAR:
    return "Sequans Monarch Modem";
  case driver::ETHERNET:
    return "Ethernet";
  default:
    return "Unknown";
  }
}

void unified_controller_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data)
{
  UnifiedController* self = static_cast<UnifiedController*>(arg);
  self->driverEventHandler(base, id, data);
}

static void _handleIpEvent(void* handler_args, esp_event_base_t base, int32_t event_id, void* data)
{
  ESP_LOGD(LOGTAG, "Handling IP event %" PRIu32, event_id);
  UnifiedController* controller = (UnifiedController*) handler_args;
  EventBits_t bits = xEventGroupGetBits(ip_event_group);

  switch(event_id) {
  case IP_EVENT_STA_GOT_IP: {
    if(bits & STA_IP_BIT)
      break;

    ip_event_got_ip_t* event = (ip_event_got_ip_t*) data;
    esp_netif_t* netif = event->esp_netif;

    esp_netif_dns_info_t dns_main   = {};
    esp_netif_dns_info_t dns_backup = {};
    esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN,   &dns_main);
    esp_netif_get_dns_info(netif, ESP_NETIF_DNS_BACKUP, &dns_backup);

    ESP_LOGI(LOGTAG, "Station received IP address");
    ESP_LOGI(LOGTAG, "~~~~~~~~~~~~~~");
    ESP_LOGI(LOGTAG, "IP          : " IPSTR, IP2STR(&event->ip_info.ip));
    ESP_LOGI(LOGTAG, "Netmask     : " IPSTR, IP2STR(&event->ip_info.netmask));
    ESP_LOGI(LOGTAG, "Gateway     : " IPSTR, IP2STR(&event->ip_info.gw));
    ESP_LOGI(LOGTAG, "Name Server1: " IPSTR, IP2STR(&dns_main.ip.u_addr.ip4));
    ESP_LOGI(LOGTAG, "Name Server2: " IPSTR, IP2STR(&dns_backup.ip.u_addr.ip4));
    ESP_LOGI(LOGTAG, "~~~~~~~~~~~~~~");

    xEventGroupSetBits(ip_event_group, STA_IP_BIT);

    if(!controller->requiresInternetCheck(driver::WIFI)) {
      _postNetworkUpEvent(driver::WIFI, "Espressif WiFi", netif);
    }
    break;
  }
  case IP_EVENT_STA_LOST_IP: {
    if(!(bits & STA_IP_BIT))
      break;

    const bool wasSelected = controller->isSelectedDriverType(driver::WIFI);

    ESP_LOGW(LOGTAG, "Station lost IP address");
    xEventGroupClearBits(ip_event_group, STA_IP_BIT);
    controller->clearSelectedDriverIfType(driver::WIFI);

    if(wasSelected) {
      uc_network_event_t net_event = {
        .interface_type = driver::WIFI,
        .driver_name    = "Espressif WiFi",
        .ip_info        = {},
        .dns_main       = {},
        .dns_backup     = {},
      };
      esp_event_post(UC_NETWORK_EVENT_BASE, UC_EVENT_NETWORK_DOWN,
                     &net_event, sizeof(net_event), portMAX_DELAY);

      controller->triggerEvaluation();
    }
    break;
  }
  case IP_EVENT_AP_STAIPASSIGNED: {
    break;
  }
  case IP_EVENT_GOT_IP6: {
    ip_event_got_ip6_t* event6 = (ip_event_got_ip6_t*) data;
    ESP_LOGI(LOGTAG, "Got IPv6 address " IPV6STR, IPV62STR(event6->ip6_info.ip));
    break;
  }
  case IP_EVENT_ETH_GOT_IP: {
    if(bits & ETH_IP_BIT)
      break;

    ip_event_got_ip_t* event = (ip_event_got_ip_t*) data;
    esp_netif_t* netif = event->esp_netif;

    esp_netif_dns_info_t dns_main   = {};
    esp_netif_dns_info_t dns_backup = {};
    esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN,   &dns_main);
    esp_netif_get_dns_info(netif, ESP_NETIF_DNS_BACKUP, &dns_backup);

    ESP_LOGI(LOGTAG, "Ethernet received IP address");
    ESP_LOGI(LOGTAG, "~~~~~~~~~~~~~~");
    ESP_LOGI(LOGTAG, "IP          : " IPSTR, IP2STR(&event->ip_info.ip));
    ESP_LOGI(LOGTAG, "Netmask     : " IPSTR, IP2STR(&event->ip_info.netmask));
    ESP_LOGI(LOGTAG, "Gateway     : " IPSTR, IP2STR(&event->ip_info.gw));
    ESP_LOGI(LOGTAG, "Name Server1: " IPSTR, IP2STR(&dns_main.ip.u_addr.ip4));
    ESP_LOGI(LOGTAG, "Name Server2: " IPSTR, IP2STR(&dns_backup.ip.u_addr.ip4));
    ESP_LOGI(LOGTAG, "~~~~~~~~~~~~~~");

    xEventGroupSetBits(ip_event_group, ETH_IP_BIT);

    if(!controller->requiresInternetCheck(driver::ETHERNET)) {
      _postNetworkUpEvent(driver::ETHERNET, "Ethernet", netif);
    }
    break;
  }
  case IP_EVENT_ETH_LOST_IP: {
    if(!(bits & ETH_IP_BIT))
      break;

    const bool wasSelected = controller->isSelectedDriverType(driver::ETHERNET);

    ESP_LOGW(LOGTAG, "Ethernet lost IP address");
    xEventGroupClearBits(ip_event_group, ETH_IP_BIT);
    controller->clearSelectedDriverIfType(driver::ETHERNET);

    if(wasSelected) {
      uc_network_event_t net_event = {
        .interface_type = driver::ETHERNET,
        .driver_name    = "Ethernet",
        .ip_info        = {},
        .dns_main       = {},
        .dns_backup     = {},
      };
      esp_event_post(UC_NETWORK_EVENT_BASE, UC_EVENT_NETWORK_DOWN,
                     &net_event, sizeof(net_event), portMAX_DELAY);

      controller->triggerEvaluation();
    }
    break;
  }
  case IP_EVENT_PPP_GOT_IP: {
    if(bits & PPP_IP_BIT)
      break;

    ip_event_got_ip_t* event = (ip_event_got_ip_t*) data;
    esp_netif_t* netif = event->esp_netif;

    esp_netif_dns_info_t dns_main   = {};
    esp_netif_dns_info_t dns_backup = {};
    esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN,   &dns_main);
    esp_netif_get_dns_info(netif, ESP_NETIF_DNS_BACKUP, &dns_backup);

    ESP_LOGI(LOGTAG, "PPP interface received IP address");
    ESP_LOGI(LOGTAG, "~~~~~~~~~~~~~~");
    ESP_LOGI(LOGTAG, "IP          : " IPSTR, IP2STR(&event->ip_info.ip));
    ESP_LOGI(LOGTAG, "Netmask     : " IPSTR, IP2STR(&event->ip_info.netmask));
    ESP_LOGI(LOGTAG, "Gateway     : " IPSTR, IP2STR(&event->ip_info.gw));
    ESP_LOGI(LOGTAG, "Name Server1: " IPSTR, IP2STR(&dns_main.ip.u_addr.ip4));
    ESP_LOGI(LOGTAG, "Name Server2: " IPSTR, IP2STR(&dns_backup.ip.u_addr.ip4));
    ESP_LOGI(LOGTAG, "~~~~~~~~~~~~~~");

    xEventGroupSetBits(ip_event_group, PPP_IP_BIT);

    if(!controller->requiresInternetCheck(driver::CELLULAR)) {
      _postNetworkUpEvent(driver::CELLULAR, "Sequans Monarch Modem", netif);
    }
    break;
  }
  case IP_EVENT_PPP_LOST_IP: {
    if(!(bits & PPP_IP_BIT))
      break;

    const bool wasSelected = controller->isSelectedDriverType(driver::CELLULAR);

    ESP_LOGW(LOGTAG, "PPP interface lost IP address");
    xEventGroupClearBits(ip_event_group, PPP_IP_BIT);
    controller->clearSelectedDriverIfType(driver::CELLULAR);

    if(wasSelected) {
      uc_network_event_t net_event = {
        .interface_type = driver::CELLULAR,
        .driver_name    = "Sequans Monarch Modem",
        .ip_info        = {},
        .dns_main       = {},
        .dns_backup     = {},
      };
      esp_event_post(UC_NETWORK_EVENT_BASE, UC_EVENT_NETWORK_DOWN,
                     &net_event, sizeof(net_event), portMAX_DELAY);

      controller->triggerEvaluation();
    }
    break;
  }
  case IP_EVENT_TX_RX:
  default: {
    break;
  }
  }
}
} // namespace

UnifiedController::UnifiedController(sUnifiedCommInternal* boardConfig) : _board_config(boardConfig)
{
  init = true;
  ESP_LOGD(LOGTAG, "initalized unified comms for %s.", boardConfig->name.data());
}

void UnifiedController::printConfig()
{
  uint8_t offset = 0;
  driver::Driver* driver = _board_config->drivers[offset];
  while(driver != nullptr) {
    driver->printConfig();
    driver = _board_config->drivers[++offset];
  }
}

bool UnifiedController::requiresInternetCheck(driver::InterfaceType type) const
{
  if(_board_config == nullptr) {
    return false;
  }

  for(size_t i = 0; _board_config->drivers[i] != nullptr; ++i) {
    driver::Driver* candidate = _board_config->drivers[i];
    if(candidate->type == type) {
      return candidate->requiresInternetCheck();
    }
  }

  return false;
}

bool UnifiedController::isSelectedDriverType(driver::InterfaceType type) const
{
  return _selected_driver != nullptr && _selected_driver->type == type;
}

void UnifiedController::clearSelectedDriverIfType(driver::InterfaceType type)
{
  if(_selected_driver != nullptr && _selected_driver->type == type) {
    _selected_driver = nullptr;
  }
}
bool UnifiedController::start()
{
  if(_board_config->drivers[0] == nullptr) {
    ESP_LOGE(LOGTAG,
             "No drivers configured! Please configure at least one for unified comms to work.");
    return false;
  }

  vTaskDelay(pdMS_TO_TICKS(1000));

  ESP_LOGI(LOGTAG, "Started unified controller");
  if(!_createEventLoop()) {
    ESP_LOGD(LOGTAG, "unable to create controller event loop");
    return false;
  }

  // Event loop could have already been initialized
  esp_err_t ret = esp_event_loop_create_default();
  if(ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    ESP_LOGE(LOGTAG, "Unable to create event loop");
  if(ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    return false;

  ret = esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &_handleIpEvent, this);
  if(ret != ESP_OK)
    ESP_LOGE(LOGTAG, "Unable to register IP event handler");
  if(ret != ESP_OK)
    return false;

  ip_event_group = xEventGroupCreate();
  if(!ip_event_group)
    ESP_LOGE(LOGTAG, "Unable to create event group");
  if(!ip_event_group)
    return false;

  // Schedule the first evaluation before creating the task so the task sees
  // EvaluationSchedule::Soon on its very first iteration and does not suspend itself.
  _evaluation_schedule = EvaluationSchedule::Soon;

  xTaskCreate(&UnifiedController::_ucTask, "Unified Controller Task", 4096, this, 5,
              &_uc_task_handle);

  if(_uc_task_handle == nullptr) {
    ESP_LOGW(LOGTAG, "Couldn't start the unified controller task");
    return false;
  }

  return true;
}

void UnifiedController::evaluateDrivers()
{
  ESP_LOGI(LOGTAG, "Evaluating available drivers");

  driver::Driver* previousSelected = _selected_driver;
  driver::Driver* bestDriver = nullptr;

  // Helper: fire NETWORK_DOWN for the previously-selected driver exactly once,
  // as soon as we know it is being dropped.  Clears previousSelected so the
  // end-of-function guard doesn't fire a duplicate.
  auto notifyDownIfWasSelected = [&](driver::Driver* dropped) {
    if(previousSelected != nullptr && dropped == previousSelected) {
      uc_network_event_t net_event = {
        .interface_type = previousSelected->type,
        .driver_name    = _driverNameForType(previousSelected->type),
        .ip_info        = {},
        .dns_main       = {},
        .dns_backup     = {},
      };
      esp_event_post(UC_NETWORK_EVENT_BASE, UC_EVENT_NETWORK_DOWN,
                     &net_event, sizeof(net_event), portMAX_DELAY);
      previousSelected = nullptr; // consumed – prevent duplicate at end
    }
  };

  // Iterate through every driver starting with the highest priority, until one is connected
  for(driver::Driver** current = _board_config->drivers;
      current != nullptr && (*current) != nullptr && (bestDriver == nullptr); current++) {
    auto driver = *current;

    if(!driver->isConfigured()) {
      ESP_LOGW(LOGTAG, "%.*s is not configured", (int) driver->name.size(), driver->name.data());
      continue;
    }

    EventBits_t driverBit = interfaceTypeToBit(driver->type);
    if(driverBit == 0) {
      ESP_LOGW(LOGTAG, "%.*s has invalid interface type %d", (int) driver->name.size(),
               driver->name.data(), driver->type);
      continue;
    }

    // Skip if driver already has IP
    EventBits_t bits = xEventGroupGetBits(ip_event_group);
    if(bits & driverBit) {
      ESP_LOGI(LOGTAG, "%.*s already has IP, verifying", (int) driver->name.size(), driver->name.data());
      if(driver->requiresInternetCheck() &&
         !_verifyInternetReachabilityOnDriver(driver)) {
        ESP_LOGW(LOGTAG,
                 "%.*s has IP but failed internet check – skipping",
                 (int) driver->name.size(), driver->name.data());
        notifyDownIfWasSelected(driver);
        xEventGroupClearBits(ip_event_group, driverBit);
        driver->disconnect();
        continue;
      }
      bestDriver = driver;
      continue;
    }

    if(!driver->connect()) {
      ESP_LOGW(LOGTAG, "%.*s failed to connect", (int) driver->name.size(), driver->name.data());
      notifyDownIfWasSelected(driver);
      driver->disconnect();
      continue;
    }

    // Once the driver reports it is connected, the IP should arrive quickly.
    // Cap the wait at 10 s – if DHCP/PPP negotiation hasn't finished by then
    // the interface is considered unusable and we fall through to the next driver.
    constexpr uint32_t IP_ACQUIRE_TIMEOUT_S = 10;
    ESP_LOGI(LOGTAG, "%.*s waiting up to %" PRIu32 " s for an IP",
             (int) driver->name.size(), driver->name.data(), IP_ACQUIRE_TIMEOUT_S);
    bits = xEventGroupWaitBits(ip_event_group, driverBit, pdFALSE, pdTRUE,
                               pdMS_TO_TICKS(IP_ACQUIRE_TIMEOUT_S * 1000));

    if(bits & driverBit) {
      ESP_LOGI(LOGTAG, "%.*s connected", (int) driver->name.size(), driver->name.data());
      if(driver->requiresInternetCheck() &&
         !_verifyInternetReachabilityOnDriver(driver)) {
        ESP_LOGW(LOGTAG,
                 "%.*s failed internet check after IP acquired – skipping",
                 (int) driver->name.size(), driver->name.data());
        notifyDownIfWasSelected(driver);
        xEventGroupClearBits(ip_event_group, driverBit);
        driver->disconnect();
        continue;
      }

      bestDriver = driver;
    } else {
      ESP_LOGW(LOGTAG, "%.*s timed out waiting for an IP – skipping", (int) driver->name.size(),
               driver->name.data());
      notifyDownIfWasSelected(driver);
      driver->disconnect();
    }
  }

  // If the selected driver is changing (including going to nullptr), notify the
  // application that the previous interface is no longer available.
  if(previousSelected != nullptr && bestDriver != previousSelected) {
    uc_network_event_t net_event = {
      .interface_type = previousSelected->type,
      .driver_name    = _driverNameForType(previousSelected->type),
      .ip_info        = {},
      .dns_main       = {},
      .dns_backup     = {},
    };
    esp_event_post(UC_NETWORK_EVENT_BASE, UC_EVENT_NETWORK_DOWN,
                   &net_event, sizeof(net_event), portMAX_DELAY);
  }

  if(bestDriver != nullptr && bestDriver->requiresInternetCheck() &&
     bestDriver != previousSelected) {
    _postNetworkUpEvent(bestDriver->type,
                        _driverNameForType(bestDriver->type),
                        bestDriver->getInterface());
  }

  _selected_driver = bestDriver;
}

void UnifiedController::deactivateUnselectedDrivers()
{
  driver::Driver** current = _board_config->drivers;

  while(current != nullptr && (*current) != nullptr) {
    auto driver = *current;

    if(driver == _selected_driver) {
      current++;
      continue;
    }

    if(driver->isConfigured()) {
      bool success = driver->disconnect();
      if(!success) {
        ESP_LOGW(LOGTAG, "Couldn't disconnect driver %.*s", (int) driver->name.size(),
                 driver->name.data());
      }
    }

    current++;
  }
}

esp_err_t UnifiedController::registerEventHandler(driver::EventType event_id,
                                                  esp_event_handler_t event_handler,
                                                  void* event_handler_arg)
{
  return esp_event_handler_register(UC_DRIVER_BASE, event_id, event_handler, event_handler_arg);
}

esp_err_t UnifiedController::registerNetworkEventHandler(uc_event_t event_id,
                                                         esp_event_handler_t handler,
                                                         void* handler_arg)
{
  return esp_event_handler_register(UC_NETWORK_EVENT_BASE, event_id, handler, handler_arg);
}

void UnifiedController::_sortDrivers()
{
  if(_board_config != nullptr) {
    /* First, find number of drivers */
    size_t count = 0;
    while(_board_config->drivers[count] != nullptr) {
      ++count;
    }

    /* Now sort them in-place */
    std::sort(_board_config->drivers, _board_config->drivers + count, driver::DriverComparator {});

    // Sort drivers based on priority
    ESP_LOGD(LOGTAG, "Sorted drivers");
  }
}

bool UnifiedController::_createEventLoop()
{
  if(_event_loop) {
    /* destroy previous event loop when applicable */
    _destroyEventLoop();
  }
  esp_event_loop_args_t loopArgs = {
    .queue_size = 12,
    .task_name = "UnifiedController",
    .task_priority = 5,
    .task_stack_size = 4096,
    .task_core_id = 0 /* let ESP choose*/
  };

  esp_err_t eventCreateRes = esp_event_loop_create(&loopArgs, &_event_loop);

  if(eventCreateRes != ESP_OK) {
    ESP_LOGE(LOGTAG, "unable to create the Unified Event Loop: esp_err_t(%i)", eventCreateRes);
    return false;
  }

  size_t count = 0;
  while(_board_config->drivers[count] != nullptr) {
    if(_board_config->drivers[count]) {
      _board_config->drivers[count]->eventLoop = _event_loop;
    }
    ++count;
  }

  esp_event_handler_register_with(_event_loop, UC_DRIVER_BASE, ESP_EVENT_ANY_ID,
                                  unified_controller_event_handler, this);

  ESP_LOGD(LOGTAG, "created Unified Event Loop");
  return true;
}

void UnifiedController::_destroyEventLoop()
{
  if(_event_loop) {
    esp_err_t eventDestroyRes = esp_event_loop_delete(_event_loop);

    if(eventDestroyRes != ESP_OK) {
      ESP_LOGE(LOGTAG, "unable to destroy default event loop");
    } else {
      _event_loop = nullptr;
    }
  }
}

void UnifiedController::triggerEvaluation()
{
  _evaluation_schedule = EvaluationSchedule::Soon;

  // Wake the evaluation task if it is suspended or waiting
  vTaskResume(_uc_task_handle);
  xTaskNotifyGive(_uc_task_handle);
}

void UnifiedController::_ucTask(void* pvParameters)
{
  UnifiedController* self = static_cast<UnifiedController*>(pvParameters);

  while(true) {
    if(self->_evaluation_schedule == EvaluationSchedule::Idle) {
      vTaskSuspend(nullptr); // Suspend until an evaluation is triggered
      continue;
    }

    self->_sortDrivers();
    self->evaluateDrivers();

    driver::Driver* drv = self->_selected_driver;

    if(drv != nullptr) {
      const bool isHighestPriority = drv->getPriority() == self->_board_config->drivers[0]->getPriority();
      const bool needsPeriodicCheck = !isHighestPriority || drv->requiresInternetCheck();

      if(needsPeriodicCheck) {
        // Re-evaluate periodically either because a higher-priority driver may become
        // available, or because this driver requires ongoing internet reachability checks.
        self->_evaluation_schedule = EvaluationSchedule::Periodic;
      } else {
        // Highest-priority driver active with no internet check – nothing to re-evaluate.
        self->_evaluation_schedule = EvaluationSchedule::Idle;
      }

      ESP_LOGI(LOGTAG,
               " === Selected [ %.*s ] as active network driver === ", (int) drv->name.size(),
               drv->name.data());

      // Log the next evaluation schedule immediately after selection, before the potentially
      // slow deactivateUnselectedDrivers() call so it appears at the right point in the log.
      switch(self->_evaluation_schedule) {
      case EvaluationSchedule::Soon:
        ESP_LOGI(LOGTAG, "Next driver evaluation in 20 s");
        break;
      case EvaluationSchedule::Periodic:
        ESP_LOGI(LOGTAG, "Next driver evaluation in 5 min");
        break;
      default:
        ESP_LOGI(LOGTAG, "Highest-priority driver active – no periodic re-evaluation scheduled");
        break;
      }

      esp_netif_set_default_netif(drv->getInterface());
      self->deactivateUnselectedDrivers();

    } else {
      // No driver is usable – schedule a retry soon
      self->_evaluation_schedule = EvaluationSchedule::Soon;
      ESP_LOGW(LOGTAG, "No driver could provide a network connection. Retrying soon...");
    }

    // Wait for the next scheduled evaluation, but allow early wake via notification
    TickType_t delay_ticks = 0;
    switch(self->_evaluation_schedule) {
    case EvaluationSchedule::Soon:
      delay_ticks = pdMS_TO_TICKS(20 * 1000);
      break;
    case EvaluationSchedule::Periodic:
      delay_ticks = pdMS_TO_TICKS(5 * 60 * 1000);
      break;
    default:
      delay_ticks = portMAX_DELAY;
      break;
    }

    // Wait, but allow immediate wake via notification
    ulTaskNotifyTake(pdTRUE, delay_ticks);
  }
}

void UnifiedController::driverEventHandler(esp_event_base_t event_base, int32_t event_id,
                                           void* event_data)
{

  driver::InterfaceType* iftype = (driver::InterfaceType*) event_data;

  switch(event_id) {
  case driver::CONNECTING: {
    if(*iftype == driver::InterfaceType::CELLULAR) {

    } else if(*iftype == driver::InterfaceType::WIFI) {

    } else if(*iftype == driver::InterfaceType::ETHERNET) {
    }
    break;
  }

  case driver::DISCONNECTING: {
    if(*iftype == driver::InterfaceType::CELLULAR) {
      if(!(xEventGroupGetBits(ip_event_group) & PPP_IP_BIT))
        break;

      const bool wasSelected = isSelectedDriverType(driver::CELLULAR);
      ESP_LOGW(LOGTAG, "Cellular interface lost connection");

      xEventGroupClearBits(ip_event_group, PPP_IP_BIT);
      clearSelectedDriverIfType(driver::CELLULAR);

      if(wasSelected) {
        uc_network_event_t net_event = {
          .interface_type = driver::CELLULAR,
          .driver_name    = _driverNameForType(driver::CELLULAR),
        };
        esp_event_post(UC_NETWORK_EVENT_BASE, UC_EVENT_NETWORK_DOWN,
                       &net_event, sizeof(net_event), portMAX_DELAY);
        triggerEvaluation();
      }
    } else if(*iftype == driver::InterfaceType::WIFI) {
      if(!(xEventGroupGetBits(ip_event_group) & STA_IP_BIT))
        break;

      const bool wasSelected = isSelectedDriverType(driver::WIFI);
      ESP_LOGW(LOGTAG, "Wi-Fi interface lost connection");

      xEventGroupClearBits(ip_event_group, STA_IP_BIT);
      clearSelectedDriverIfType(driver::WIFI);

      if(wasSelected) {
        uc_network_event_t net_event = {
          .interface_type = driver::WIFI,
          .driver_name    = _driverNameForType(driver::WIFI),
        };
        esp_event_post(UC_NETWORK_EVENT_BASE, UC_EVENT_NETWORK_DOWN,
                       &net_event, sizeof(net_event), portMAX_DELAY);
        triggerEvaluation();
      }
    } else if(*iftype == driver::InterfaceType::ETHERNET) {
      if(!(xEventGroupGetBits(ip_event_group) & ETH_IP_BIT))
        break;

      const bool wasSelected = isSelectedDriverType(driver::ETHERNET);
      ESP_LOGW(LOGTAG, "Ethernet interface lost connection");

      xEventGroupClearBits(ip_event_group, ETH_IP_BIT);
      clearSelectedDriverIfType(driver::ETHERNET);

      if(wasSelected) {
        uc_network_event_t net_event = {
          .interface_type = driver::ETHERNET,
          .driver_name    = _driverNameForType(driver::ETHERNET),
        };
        esp_event_post(UC_NETWORK_EVENT_BASE, UC_EVENT_NETWORK_DOWN,
                       &net_event, sizeof(net_event), portMAX_DELAY);
        triggerEvaluation();
      }
    }
    break;
  }

  default: {
    ESP_LOGD(LOGTAG, "Unknown driver event");
    break;
  }
  }
}