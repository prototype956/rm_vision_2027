# ============================================================================
# 当前启用的源码模块
# ============================================================================

message(STATUS "  Configuring source modules...")

add_subdirectory(core)
add_subdirectory(geometry)
add_subdirectory(simulation)
add_subdirectory(frame)
add_subdirectory(tool)

if(USE_ARMOR_DETECTOR)
    add_subdirectory(modules)
endif()

add_subdirectory(hal)

# 供相机标定和视觉运行时共用；BUILD_MAIN=OFF 时也可加载实机部署配置。
add_library(mv-real-geometry-config STATIC runtime/real_geometry_config.cpp)
target_include_directories(mv-real-geometry-config PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(mv-real-geometry-config PUBLIC mv-hal-imu mv-core)

if(USE_ARMOR_DETECTOR)
    include(${PROJECT_SOURCE_DIR}/cmake/GimbalControlRuntime.cmake)
endif()

if(BUILD_MAIN)
    add_subdirectory(runtime)
    add_subdirectory(app)
endif()
