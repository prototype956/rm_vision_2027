add_subdirectory(camera)

if(USE_OPENVINO)
    add_subdirectory(armor_detector)
endif()

if(BUILD_MAIN AND USE_OPENVINO)
    add_subdirectory(runtime)
    add_subdirectory(web)
endif()
