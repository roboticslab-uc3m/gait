# Copyright: Universidad Carlos III de Madrid (C) 2016;
# Authors: munozyanez
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)

project(Gait)

if(MSVC)
    message(STATUS "Running on windows")    
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

# Let the user specify a configuration (only single-config generators).
if(NOT CMAKE_CONFIGURATION_TYPES)
    # Possible values.
    set(_configurations Debug Release MinSizeRel RelWithDebInfo)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${_configurations})

    foreach(_conf ${_configurations})
        set(_conf_string "${_conf_string} ${_conf}")
    endforeach()

    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY HELPSTRING
                 "Choose the type of build, options are:${_conf_string}")

    if(NOT CMAKE_BUILD_TYPE)
        # Encourage the user to specify build type.
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE Release)
    endif()
endif()

# Standard installation directories.
include(GNUInstallDirs)

# Control where libraries and executables are placed during the build.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})

# Pick up our cmake modules - they are all in the cmake subdirectory.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# Enabling coverage.
option(ENABLE_coverage "Choose if you want to enable coverage collection" OFF)

if(ENABLE_coverage)
    # List supported compilers.
    if(CMAKE_COMPILER_IS_GNUCXX)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
    endif()
endif()

# Add main contents.
add_subdirectory(src)

if(NOT GAIT_PART_OF_PROJECT)

# CMake installation path.
if(WIN32 AND NOT CYGWIN)
    set(GAIT_CMAKE_DESTINATION CMake)
else()
    set(GAIT_CMAKE_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gait)
endif()

# Add all targets to the build-tree export set.
export(TARGETS Gait
       NAMESPACE ROBOTICSLAB::
       FILE GAITTargets.cmake)

# Store the package in the user registry.
export(PACKAGE Gait)

# Generate simple <pkg>Config.cmake file.
file(WRITE ${CMAKE_BINARY_DIR}/GAITConfig.cmake "include(\${CMAKE_CURRENT_LIST_DIR}/GAITTargets.cmake)")

# Install GAITConfig.cmake
install(FILES ${CMAKE_BINARY_DIR}/GAITConfig.cmake
        DESTINATION ${GAIT_CMAKE_DESTINATION})

# Install the export set for use with the install-tree.
install(EXPORT GAIT_EXPORT
        NAMESPACE ROBOTICSLAB::
        FILE GAITTargets.cmake
        DESTINATION ${GAIT_CMAKE_DESTINATION})

# Configure and create uninstall target.
include(AddUninstallTarget)

endif(NOT GAIT_PART_OF_PROJECT)
