/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <memory>
#include <future>
#define CATCH_CONFIG_DISABLE_EXCEPTIONS
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_session.hpp>
#include <cxx_include/InterfaceConfig.hpp>


#include <esp_netif.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <esp_log.h>
#define CATCH_CONFIG_MAIN

using namespace unifiedComms;

//stop the netif
void reset()
{
    // Deinitialize Wi-Fi if it's running
    esp_wifi_stop();
    esp_wifi_deinit();

    // Deinitialize network interfaces
    esp_netif_deinit();
    nvs_flash_deinit();
}


RTC_DATA_ATTR static uint8_t crash_counter = 0;
#define CATCH_CONFIG_RUNNER
extern "C" int app_main(void)
{
    if (crash_counter > 0) {
        ESP_LOGW("", "Previous startup ended in a crash. Skipping Catch2 tests.");
        return -1;
    }
    crash_counter = 1;
    // Run the Catch2 session and store the result
    int result = Catch::Session().run();

    // Use more descriptive error handling
    if (result != 0) {
        printf("Test failed with result %d.\n", result);
    } else {
        printf("All tests passed successfully.\n");
    }
    crash_counter = 0;
    // Exit the application with the test result as the status code
    return result;
}