cmake_minimum_required(VERSION 3.16)
set(
    EXTRA_COMPONENT_DIRS
        "src"
        "${CMAKE_SOURCE_DIR}/../s3_common"
)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(s3_slave)
