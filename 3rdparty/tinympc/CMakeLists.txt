##
## TinyMPC 静态库 (mv-3rdparty-tinympc)
##
## 来源：从 sp_vision_25/tasks/auto_aim/planner/tinympc/ 复制
## 适配：去除 sp 私有 include 路径，改用 Eigen3::Eigen CMake 目标
##
## 外部只需 target_link_libraries(target PRIVATE mv-3rdparty-tinympc)，
## Eigen 头文件和 tinympc/ 头文件路径均自动传递。
##

add_library(mv-3rdparty-tinympc STATIC
    admm.cpp
    tiny_api.cpp
    codegen.cpp
    rho_benchmark.cpp
)

set_property(TARGET mv-3rdparty-tinympc PROPERTY POSITION_INDEPENDENT_CODE ON)

target_link_libraries(mv-3rdparty-tinympc
    PUBLIC
        Eigen3::Eigen
)

# PUBLIC: 让使用者可以直接 #include "tinympc/tiny_api.hpp"（以 3rdparty 为根）
target_include_directories(mv-3rdparty-tinympc
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..   # 使用者 include tinympc/tiny_api.hpp
)

target_compile_options(mv-3rdparty-tinympc
    PRIVATE
        -w     # 屏蔽第三方警告，不影响我们自己代码的警告策略
)
