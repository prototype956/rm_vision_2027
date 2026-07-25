# ============================================================================
# mv-factory — 模板工厂注册表（纯头文件库）
# ============================================================================
#
# factory.hpp 全部是模板代码，编译期展开，不需要单独编译的翻译单元，
# 使用 INTERFACE 库传播 include 路径即可。

add_library(mv-factory INTERFACE)

target_include_directories(mv-factory
    INTERFACE
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(mv-factory
    INTERFACE
        mv-interfaces   # 工厂宏中的各注册宏需要知道 ICamera/ISerial 所在命名空间
        mv-hal-camera   # MV_REGISTER_CAMERA 宏引用 mv::hal::ICamera
        mv-hal-serial   # MV_REGISTER_SERIAL 宏引用 mv::hal::ISerial
)

message(STATUS "    [factory] mv-factory (INTERFACE, header-only)")
