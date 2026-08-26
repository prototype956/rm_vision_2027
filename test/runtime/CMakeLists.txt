add_executable(mv-runtime-policy-test
    runtime_policy_test_main.cpp
)

target_link_libraries(mv-runtime-policy-test
    PRIVATE
        mv-runtime
)

set_target_properties(mv-runtime-policy-test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

message(STATUS "    + mv-runtime-policy-test (fault policy + safe stop acceptance)")
