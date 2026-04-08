/**
 * @file driver.hpp
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

#pragma once
#ifndef _DRIVER_
#define _DRIVER_

#ifndef ADD_DRIVER
#define ADD_DRIVER(driver_name) (driver::driver*) (&driver_name)
#endif

#include <string_view>
#include <esp_netif.h>
#include <esp_event_base.h>

ESP_EVENT_DEFINE_BASE(UC_DRIVER_BASE);

/**
 * @brief This is the namespace containing all Driver related Code, such as:
 * driver interface, wifi driver interface, cellular driver interface
 */
namespace driver {
/**
 * @brief The types of network interfaces supported by unified comms
 */
enum InterfaceType {
  NONE,
  CELLULAR, // PPP
  WIFI,     // ESP
  ETHERNET,
  DRIVER_TYPE_COUNT,
};

enum EventType : int32_t { DISCONNECTING, CONNECTING };

enum StatusType : uint8_t { DISCONNECTED, CONNECTED };

/**
 * @brief The common interface for all drivers.
 *
 * this interface is used by the unified controller.
 *
 * it contains all the neccesary controll infrastructure such that auto failover can happen beteween
 * drivers as long as the interface requriements are properly met
 */
class Driver
{
protected:
  friend struct DriverComparator;
  friend class UnifiedController;
  esp_netif_t* network_interface;
  int      priority;

  /** Maximum time to wait for an IP address after connect() returns true.
   *  Defaults to 10 s; overridden by the driver-specific config() call. */
  uint32_t connectionTimeoutMs = 10000;

public:
  esp_event_loop_handle_t eventLoop = nullptr; // handle to the unifiedController eventLoop (filled
                                               // in by the unified controller on startup)
  std::string_view name;
  InterfaceType type = DRIVER_TYPE_COUNT;
  StatusType status = DISCONNECTED;

  /**
   * @brief this function returns the configured driver priority or 0 when not configured
   */
  int getPriority() { return priority; }

  /**
   * @brief Returns the maximum time in milliseconds the unified controller will
   * wait for this driver to obtain an IP address after calling connect().
   *
   * The default is 10 000 ms (10 s).  Slow links such as LTE-M/NB-IoT may
   * need several minutes; fast links such as Wi-Fi are usually done in under
   * 20 s.  The value is set via the driver-specific @c config() call.
   */
  uint32_t getConnectionTimeoutMs() const { return connectionTimeoutMs; }

  virtual esp_netif_t* getInterface() { return network_interface; };

  /**
   * @brief destroys the driver and all its internall structures.
   */
  virtual void destroy() = 0;

  /**
   * @brief prints the driver configuration
   */
  virtual void printConfig() = 0;

  /**
   * @brief This function attempts to connect the driver to the network.
   *
   * @note a valid connection is one where the netif_t has a valid IP-ADDRESS
   */
  virtual bool connect() = 0;

  /**
   * @brief This function attempts to connect the driver to the network.
   *
   * @note a valid connection is one where the netif_t has a valid IP-ADDRESS
   */
  virtual bool disconnect() = 0;

  /**
   * @brief has the driver been configured yet?
   */
  virtual bool isConfigured() = 0;

  /**
   * @brief launch the driver disconnected event, to notify the controller.
   */
  virtual void launchDisconnectedEvent() = 0;

  /**
   * @brief launch the driver connected event, to notify the
   * end user.
   */
  virtual void launchConnectedEvent() = 0;

  virtual ~Driver() = default;
};

/**
 * @brief this struct is used to compare the drivers for sorting.
 */
struct DriverComparator {
public:
  bool operator()(driver::Driver* a, driver::Driver* b) const { return a->priority < b->priority; }
};
} // namespace driver

/* =========================================================================
 * Application-level network events (UC_NETWORK_EVENT_BASE)
 * =========================================================================
 *
 * Subscribe to these events to be notified when any managed network
 * interface gains or loses its IP address.  This is the recommended hook
 * for (re)establishing sockets and protocol sessions after a driver switch.
 *
 * Registration example:
 *
 *   uc.controller.registerNetworkEventHandler(
 *       UC_EVENT_NETWORK_UP,
 *       [](void* arg, esp_event_base_t, int32_t, void* data) {
 *           auto* e = static_cast<uc_network_event_t*>(data);
 *           ESP_LOGI(TAG, "Network up on %s – IP " IPSTR,
 *                    e->driver_name, IP2STR(&e->ip_info.ip));
 *           // (Re)connect your sockets / MQTT / CoAP sessions here.
 *       }, nullptr);
 *
 *   uc.controller.registerNetworkEventHandler(
 *       UC_EVENT_NETWORK_DOWN,
 *       [](void* arg, esp_event_base_t, int32_t, void* data) {
 *           auto* e = static_cast<uc_network_event_t*>(data);
 *           ESP_LOGW(TAG, "Network down on %s – closing sessions",
 *                    e->driver_name);
 *           // Close sockets / stop clients here.
 *       }, nullptr);
 */

/**
 * @brief Event base for application-level network-state notifications.
 *
 * Events are dispatched on the default ESP event loop so handlers can be
 * registered with standard ESP-IDF APIs as well as via
 * @c UnifiedController::registerNetworkEventHandler().
 */
ESP_EVENT_DECLARE_BASE(UC_NETWORK_EVENT_BASE);

/**
 * @brief Event IDs dispatched on @c UC_NETWORK_EVENT_BASE.
 */
enum uc_event_t : int32_t {
  /** A managed interface obtained an IP address and is ready for use.
   *  Event data: non-null pointer to @c uc_network_event_t. */
  UC_EVENT_NETWORK_UP = 0,

  /** A managed interface lost its IP address.  Re-establish any open
   *  sockets or protocol sessions before using the network again.
   *  Event data: pointer to @c uc_network_event_t; only
   *  @c interface_type and @c driver_name are valid on DOWN events. */
  UC_EVENT_NETWORK_DOWN = 1,
};

/**
 * @brief Payload carried by @c UC_NETWORK_EVENT_BASE events.
 *
 * The struct is stack-allocated inside the event dispatch path; the
 * pointer is only valid for the duration of the callback invocation.
 * Copy any fields you need to retain beyond the callback.
 */
struct uc_network_event_t {
  /** The interface type that changed state. */
  driver::InterfaceType interface_type;

  /** Human-readable driver name (null-terminated). */
  const char* driver_name;

  /** IP configuration assigned to the interface (UP only). */
  esp_netif_ip_info_t ip_info;

  /** Primary DNS server address (UP only). */
  esp_ip4_addr_t dns_main;

  /** Secondary / backup DNS server address (UP only). */
  esp_ip4_addr_t dns_backup;
};

#endif