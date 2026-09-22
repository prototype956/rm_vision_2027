add_library(mv-prediction-model STATIC
    ../armor_predictor/detail/armor_motion_model.cpp
    ../armor_predictor/prediction_extrapolation.cpp)
set_property(TARGET mv-prediction-model PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(mv-prediction-model PUBLIC ${CMAKE_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS})
# 运动模型的公共头文件包含 ceres/jet.h；同时传递导入目标的头文件搜索路径，
# 使安装在工作区中的 Ceres 也能被正确使用。
target_link_libraries(mv-prediction-model PUBLIC Eigen3::Eigen opencv_core Ceres::ceres)
