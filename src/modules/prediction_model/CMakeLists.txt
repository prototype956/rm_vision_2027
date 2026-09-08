add_library(mv-prediction-model STATIC
    ../armor_predictor/detail/armor_motion_model.cpp
    ../armor_predictor/prediction_extrapolation.cpp)
set_property(TARGET mv-prediction-model PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(mv-prediction-model PUBLIC ${CMAKE_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
target_link_libraries(mv-prediction-model PUBLIC Eigen3::Eigen opencv_core)
