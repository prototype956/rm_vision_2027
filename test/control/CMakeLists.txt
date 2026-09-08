add_executable(mv-control-reference reference.cpp)
target_compile_definitions(mv-control-reference PRIVATE PHASE1_NEW)
target_link_libraries(mv-control-reference PRIVATE mv-modules-fire-control)
add_executable(mv-control-acceptance acceptance.cpp)
target_link_libraries(mv-control-acceptance PRIVATE mv-modules-fire-control)

add_executable(mv-control-session-reference session_reference.cpp)
target_compile_definitions(mv-control-session-reference PRIVATE PHASE1_NEW)
target_link_libraries(mv-control-session-reference PRIVATE mv-modules-fire-control)
