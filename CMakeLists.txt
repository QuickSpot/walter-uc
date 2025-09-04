idf_component_register(SRCS "src/uc.cpp"
                            "src/driver/sequansGM02S/gm02s.cpp"
                            "src/driver/espWifi/espWifi.cpp"
                    INCLUDE_DIRS "src"
                    REQUIRES esp_netif esp_wifi nvs_flash esp_modem driver esp_timer)

# set(COMPONENT_NAME "walter-comm")

add_compile_options(-Wno-error)
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u _g_esp_netif_netstack_default_ppp")

set_target_properties(${COMPONENT_LIB} PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
)