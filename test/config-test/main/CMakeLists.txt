idf_component_register(SRCS "test_config.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES WHOLE_ARCHIVE)

target_link_libraries(${COMPONENT_LIB} PRIVATE Catch2WithMain)
# Set the C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)
