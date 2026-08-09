# ============================================================================
# mv-geometry — 内部三维几何类型与刚体变换
# ============================================================================

add_library(mv-geometry INTERFACE)

target_include_directories(mv-geometry
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-geometry INTERFACE Eigen3::Eigen)

message(STATUS "    + mv-geometry (Eigen 3D geometry)")
