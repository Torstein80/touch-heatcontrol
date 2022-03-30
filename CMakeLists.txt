# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)


# get_filename_component(
#     EXTRA_COMPONENT_DIRS
#     "/esp-idf-lib" DIRECTORY
#     "/u8g2" DIRECTORY
# )
# list(APPEND IDF_EXTRA_COMPONENT_DIRS ${EXTRA_COMPONENT_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/components)
cmake_path(SET EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/components/esp-idf-lib/components)

# cmake_path(APPEND EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR} "/esp-idf-lib/components")
# APPEND (EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/u8g2)
# set (path1 ${CMAKE_CURRENT_SOURCE_DIR}/esp-idf-lib/components)
# APPEND path2 ${CMAKE_CURRENT_SOURCE_DIR}/u8g2)

#set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/esp-idf-lib/components/u8g2)

#list(APPEND EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}"/u8g2" ${CMAKE_CURRENT_SOURCE_DIR}"/esp-idf-lib/components")
# set(COMPONENT_ADD_INCLUDEDIRS "/home/torstein/Documents/touch1.1/touch_element_waterproof/components/u8g2" "/home/torstein/Documents/touch1.1/touch_element_waterproof/components/esp-idf-lib")
# idf_component_register(SRCS INCLUDE_DIRS "." "components/u8g2" "components/esp-idf-lib/components") #"waterproof_example_main.c" 

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(touch_element_waterproof)

#littlefs_create_partition_image(littlefs flash_data)