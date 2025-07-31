project(SOSLAB_DriveWorks_plugin)

#-------------------------------------------------------------------------------
# License Information
#-------------------------------------------------------------------------------
# This project is licensed under the MIT License
# Copyright (c) 2025 SOSLAB Inc.
# See LICENSE file for full license text.


#-------------------------------------------------------------------------------
# Project files
#-------------------------------------------------------------------------------
set(SOURCES
    include/SOSLABLidar_Properties.hpp
    include/SOSLABLidar.hpp
    src/SOSLABLidar.cpp
    SOSLABLidarPlugin.cpp
)

set(LIBRARIES
    ${Driveworks_LIBRARIES}
    sample_sensors_plugin_common
)

#-------------------------------------------------------------------------------
# Final target
#-------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC
                            ${CMAKE_CURRENT_SOURCE_DIR}/include)
#-------------------------------------------------------------------------------
# Install target
#-------------------------------------------------------------------------------
sdk_install_sample_library(${PROJECT_NAME})
