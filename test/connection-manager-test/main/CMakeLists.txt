idf_component_register(SRCS "test_connection_manager.cpp" "test_config.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES WHOLE_ARCHIVE)

target_link_libraries(${COMPONENT_LIB} PRIVATE Catch2WithMain)

set_target_properties(${COMPONENT_LIB} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
)