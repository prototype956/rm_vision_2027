add_library(mv-prediction-model STATIC
    ../armor_predictor/detail/armor_motion_model.cpp
    ../armor_predictor/prediction_extrapolation.cpp)
set_property(TARGET mv-prediction-model PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(mv-prediction-model PUBLIC ${CMAKE_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS})
# Public motion-model headers include ceres/jet.h. Propagate the imported target's
# include paths as well, so a workspace-local Ceres installation works correctly.
target_link_libraries(mv-prediction-model PUBLIC Eigen3::Eigen opencv_core Ceres::ceres)
