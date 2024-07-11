cmake_minimum_required(VERSION 3.16)
project(RGLServerPlugin)
set(CMAKE_CXX_STANDARD 20)

if("$ENV{GZ_VERSION}" STREQUAL "garden")
    find_package(gz-sim7 REQUIRED)
    find_package(gz-plugin2 REQUIRED COMPONENTS register)

    set(GZ_TARGET_PREFIX gz)
    set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})
    set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

    message(STATUS "Compiling against Gazebo Garden")
elseif("$ENV{GZ_VERSION}" STREQUAL "harmonic")
    find_package(gz-sim8 REQUIRED)
    find_package(gz-plugin2 REQUIRED COMPONENTS register)

    set(GZ_TARGET_PREFIX gz)
    set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
    set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

    message(STATUS "Compiling against Gazebo Harmonic")
else()
    message(FATAL_ERROR "This branch is only compatible with Gazebo Garden forward.")
endif()


include_directories(include)

# Use this variable to make sure the downloaded RGL will be up-to-date
set(RGL_FORCE_DOWNLOAD OFF CACHE BOOL
    "Removes existing RGL binaries and downloads a new one")

set(RGL_TAG "v0.17.0")

set(RGL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/include/rgl")
set(RGL_SO_FILENAME "libRobotecGPULidar.so")
set(RGL_SO_PATH "${RGL_PATH}/${RGL_SO_FILENAME}")
set(RGL_API_HEADER_PATH "${RGL_PATH}/api/core.h")

set(RGL_SO_ZIP_FILENAME "RGL-core-linux-x64.zip")
set(RGL_SO_ZIP_URL "https://github.com/RobotecAI/RobotecGPULidar/releases/download/${RGL_TAG}/${RGL_SO_ZIP_FILENAME}")
set(RGL_API_HEADER_URL "https://raw.githubusercontent.com/RobotecAI/RobotecGPULidar/${RGL_TAG}/include/rgl/api/core.h")

if (RGL_FORCE_DOWNLOAD)
    file(REMOVE_RECURSE ${RGL_PATH})
endif()

# Download RGL library
if(NOT EXISTS ${RGL_SO_PATH})
    file(DOWNLOAD ${RGL_SO_ZIP_URL} ${RGL_PATH}/${RGL_SO_ZIP_FILENAME})
    file(ARCHIVE_EXTRACT INPUT ${RGL_PATH}/${RGL_SO_ZIP_FILENAME}
        DESTINATION ${RGL_PATH}
        PATTERNS ${RGL_SO_FILENAME}
        VERBOSE
    )
    file(REMOVE ${RGL_PATH}/${RGL_SO_ZIP_FILENAME})
endif()

# Download RGL API header
if(NOT EXISTS ${RGL_API_PATH})
    file(DOWNLOAD ${RGL_API_HEADER_URL} ${RGL_API_HEADER_PATH})
endif()

set(RobotecGPULidar ${RGL_SO_PATH})

add_library(RGLServerPluginManager SHARED src/RGLServerPluginManager.cc src/Mesh.cc src/Utils.cc src/Scene.cc)
target_link_libraries(RGLServerPluginManager
    PRIVATE ${GZ_TARGET_PREFIX}-sim${GZ_SIM_VER}
    PRIVATE ${GZ_TARGET_PREFIX}-plugin${GZ_PLUGIN_VER}
    ${RobotecGPULidar}
)
set_target_properties(RGLServerPluginManager PROPERTIES INSTALL_RPATH "$ORIGIN")

add_library(RGLServerPluginInstance SHARED src/RGLServerPluginInstance.cc src/Lidar.cc src/Utils.cc src/LidarPatternLoader.cc)
target_link_libraries(RGLServerPluginInstance
    PRIVATE ${GZ_TARGET_PREFIX}-sim${GZ_SIM_VER}
    PRIVATE ${GZ_TARGET_PREFIX}-plugin${GZ_PLUGIN_VER}
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
