cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

# This is required for Qt
if(POLICY CMP0100)
    cmake_policy(SET CMP0100 NEW)
endif()

project(RGLVisualize)

if("$ENV{GZ_VERSION}" STREQUAL "garden")
    find_package(gz-rendering7 REQUIRED)
    find_package(gz-gui7 REQUIRED)
    find_package(gz-msgs9 REQUIRED)

    set(GZ_TARGET_PREFIX gz)
    set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})
    set(GZ_GUI_VER ${gz-gui7_VERSION_MAJOR})
    set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})


    message(STATUS "Compiling against Gazebo Garden")
elseif("$ENV{GZ_VERSION}" STREQUAL "harmonic")
    find_package(gz-rendering8 REQUIRED)
    find_package(gz-gui8 REQUIRED)
    find_package(gz-msgs10 REQUIRED)

    set(GZ_TARGET_PREFIX gz)
    set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})
    set(GZ_GUI_VER ${gz-gui8_VERSION_MAJOR})
    set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

    message(STATUS "Compiling against Gazebo Harmonic")
else()
    message(FATAL_ERROR "This branch is only compatible with Gazebo Garden forward.")
endif()

set(CMAKE_AUTOMOC ON)

QT5_ADD_RESOURCES(resources_RCC RGLVisualize.qrc)

add_library(RGLVisualize SHARED
    RGLVisualize.cc
    ${resources_RCC}
)

target_link_libraries(RGLVisualize
    PRIVATE
    ${GZ_TARGET_PREFIX}-gui${GZ_GUI_VER}
    ${GZ_TARGET_PREFIX}-rendering${GZ_RENDERING_VER}
    ${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}
)
