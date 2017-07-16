# Copyright: Universidad Carlos III de Madrid (C) 2016;
# Authors: munozyanez
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)

# Discard any value previously set.
unset(GAIT_PART_OF_PROJECT)

if(NOT ${PROJECT_NAME} STREQUAL "Project")
    message(STATUS "Gait part of existing project: ${PROJECT_NAME}")
    set(GAIT_PART_OF_PROJECT "")
else()
    project(Gait)
endif()

# Pick up our cmake modules - they are all in the cmake subdirectory.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

if(NOT DEFINED GAIT_PART_OF_PROJECT)
    # Define installation paths and build options.
    include(GAITSystem)
    
    # Enabling coverage.
    option(ENABLE_coverage "Choose if you want to enable coverage collection" OFF)
    
    if(ENABLE_coverage)
        # List supported compilers.
        if(CMAKE_COMPILER_IS_GNUCXX)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
        endif()
    endif()
endif()

# Add main contents.
add_subdirectory(src)

if(NOT DEFINED GAIT_PART_OF_PROJECT)
    # Make this project discoverable by other projects.
    include(GAITDescribe)
    
    # Configure and create uninstall target.
    include(AddUninstallTarget)
endif()
