add_subdirectory(camera)
add_subdirectory(talos)
add_subdirectory(imu)

if(USE_ARMOR_DETECTOR)
    add_subdirectory(armor_detector)
endif()

if(BUILD_MAIN AND USE_ARMOR_DETECTOR)
    add_subdirectory(runtime)
    add_subdirectory(web)
endif()

if(USE_ARMOR_DETECTOR)
    add_subdirectory(control)
endif()
