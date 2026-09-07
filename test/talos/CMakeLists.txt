add_executable(mv-talos-protocol-test talos_protocol_test_main.cpp)
target_link_libraries(mv-talos-protocol-test PRIVATE mv-hal-camera mv-hal-gimbal-talos)
set_target_properties(mv-talos-protocol-test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
