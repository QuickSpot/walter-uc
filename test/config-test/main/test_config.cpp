/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define CATCH_CONFIG_MAIN

#include <memory>
#include <future>
#define CATCH_CONFIG_DISABLE_EXCEPTIONS
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_session.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>
#include <bsp/walter.hpp>


TEST_CASE("unifiedComms","[GM02S.config]")
{
    REQUIRE(CELL_DRV(uc.GM02S)->config("",5));
    uc.controller.printConfig();
}
std::vector<std::string> processed_urcs;

void urc_test_handler(std::string_view urc_line) {
    processed_urcs.push_back(std::string(urc_line));
}
TEST_CASE("unifiedComms","[GM02S.urc_handler]")
{
    CELL_DRV(uc.GM02S)->setUrcHandler(urc_test_handler);

    uint8_t urc_data[] = {
        '+', 'C', 'M',  'T',  'I', ':', ' ', '"', 'S',  'M', '"',
        ',', '1', '\r', '\n', // First URC: +CMTI: "SM",1\r\n
        '+', 'C', 'M',  'T',  'I', ':', ' ', '"', 'S',  'M', '"',
        ',', '2', '\r', '\n', // Second URC: +CMTI: "SM",2\r\n
        '+', 'C', 'R',  'E',  'G', ':', ' ', '0', '\r', '\n' // Third URC:
                                                             // +CREG: 0\r\n
    };

    size_t urc_len = sizeof(urc_data);

    // Call urcCallback with the concatenated URC data
    GM02S_DRV(uc.GM02S)->urcCallback(urc_data, urc_len);

    REQUIRE(processed_urcs.size() == 3);  // Should have processed 3 URCs

    REQUIRE(processed_urcs[0] == "+CMTI: \"SM\",1");
    REQUIRE(processed_urcs[1] == "+CMTI: \"SM\",2");
    REQUIRE(processed_urcs[2] == "+CREG: 0");
}

TEST_CASE("unifiedComms", "[GM02S.start]") 
{
    //CELL_DRV(uc.GM02S)->config("",5);

    REQUIRE(uc.controller.start() == true);
}
#define CATCH_CONFIG_RUNNER
extern "C" int app_main(void)
{
    // Run the Catch2 session and store the result
    int result = Catch::Session().run();

    // Use more descriptive error handling
    if (result != 0) {
        printf("Test failed with result %d.\n", result);
    } else {
        printf("All tests passed successfully.\n");
    }
    // Exit the application with the test result as the status code
    return result;
}