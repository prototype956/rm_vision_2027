add_subdirectory(camera)

if(USE_OPENVINO)
    add_subdirectory(armor_detector)
endif()
