cmake_minimum_required(VERSION 3.16)
project(RGLServerPlugin)
set(CMAKE_CXX_STANDARD 20)

if("$ENV{GZ_VERSION}" STREQUAL "garden")
    find_package(gz-sim7 REQUIRED)
    find_package(gz-plugin2 REQUIRED)

    set(GZ_TARGET_PREFIX gz)
    set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})
    set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

    message(STATUS "Compiling against Gazebo Garden")
elseif("$ENV{GZ_VERSION}" STREQUAL "harmonic")
    find_package(gz-sim8 REQUIRED)
    find_package(gz-plugin2 REQUIRED)

    set(GZ_TARGET_PREFIX gz)
    set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
    set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

    message(STATUS "Compiling against Gazebo Harmonic")
else()
    message(FATAL_ERROR "This branch is only compatible with Gazebo Garden forward.")
endif()

include_directories(include)

set(RGL_VERSION "0.16.2")
set(RGL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/include/rgl")
set(RGL_SO_PATH "${RGL_PATH}/libRobotecGPULidar.so")
set(RGL_API_PATH "${RGL_PATH}/api/core.h")
set(RGL_COMMIT "74d832f01534fc718088f8cec2509601ed08844e")
# set(RGL_SO_URL "https://github.com/RobotecAI/RobotecGPULidar/releases/download/v${RGL_VERSION}/libRobotecGPULidar.so")
set(RGL_API_URL "https://raw.githubusercontent.com/RobotecAI/RobotecGPULidar/${RGL_COMMIT}/include/rgl/api/core.h")

# You have to delete old files manually in order to download
# new ones automatically if a new version is available.
if(NOT EXISTS ${RGL_SO_PATH})
    message(FATAL_ERROR "Please copy libRobotecGPULidar.so in ${RGL_PATH}")
endif()

if(NOT EXISTS ${RGL_API_PATH})
    file(DOWNLOAD ${RGL_API_URL} ${RGL_API_PATH})
endif()

set(RobotecGPULidar ${RGL_SO_PATH})

add_library(RGLServerPluginManager SHARED src/RGLServerPluginManager.cc src/Mesh.cc src/Utils.cc src/Scene.cc)
target_link_libraries(RGLServerPluginManager
    ${GZ_TARGET_PREFIX}-sim${GZ_SIM_VER}
    ${GZ_TARGET_PREFIX}-plugin${GZ_PLUGIN_VER}
    ${RobotecGPULidar}
)
set_target_properties(RGLServerPluginManager PROPERTIES INSTALL_RPATH "$ORIGIN")

add_library(RGLServerPluginInstance SHARED src/RGLServerPluginInstance.cc src/Lidar.cc src/Utils.cc src/LidarPatternLoader.cc)
target_link_libraries(RGLServerPluginInstance
    ${GZ_TARGET_PREFIX}-sim${GZ_SIM_VER}
    ${GZ_TARGET_PREFIX}-plugin${GZ_PLUGIN_VER}
    ${RobotecGPULidar}
)
set_target_properties(RGLServerPluginInstance PROPERTIES INSTALL_RPATH "$ORIGIN")

## Install libraries

install(TARGETS RGLServerPluginInstance RGLServerPluginManager
    DESTINATION RGLServerPlugin
)
install(FILES ${RobotecGPULidar}
    DESTINATION RGLServerPlugin
)
