# gait

C++ biped gait generation library.

Requirements:

* gcc 4.6.3 or later
* CMake 2.8.12 or later

## Use as an external library

Basic lines to add/modify in CMake:

```cmake
find_package(ROBOTICSLAB_GAIT REQUIRED)
target_link_libraries(${KEYWORD} ROBOTICSLAB::Gait)
```

## Use as part of a project

Just copy this repository to the desired location within your project and traverse it with `add_subdirectory()`.

Aditionally, in [kinematics-dynamics](https://github.com/roboticslab-uc3m/kinematics-dynamics) we added these lines to `kinematics-dynamics/libraries/CMakeFiles.txt` to search for a system-available Gait, then fall back (involving GUI options) to a local copy embedded as a git submodule:

```cmake
# Try to find Gait library on system, don't fail otherwise.
find_package(ROBOTICSLAB_GAIT QUIET)

unset(HAVE_GAIT)

# Do we have Gait, either as separate installation or git submodule of current project?
if(ROBOTICSLAB_GAIT_FOUND OR EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/gait/CMakeLists.txt)
    set(HAVE_GAIT TRUE)
endif()

# Present GUI option to user if Gait was found.
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_Gait "Enable/disable Gait library" ON
                       HAVE_GAIT OFF)

# Gait enabled and not found on system, but available as submodule.
if(ENABLE_Gait AND NOT ROBOTICSLAB_GAIT_FOUND)
    message(STATUS "Using Gait library as part of current project")
    add_subdirectory(gait)
    #set_property(GLOBAL APPEND PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_TARGETS Gait)
endif()
```

An alias library target will be created to meet the same usage requirement regardless of whether Gait is found on the system or contained within your project: `target_link_libraries(${KEYWORD} ROBOTICSLAB::Gait)`.

Another example of use can be found at [gaitcontrol](https://github.com/roboticslab-uc3m/gaitcontrol), specifically at `gaitcontrol/cmake` folder.

## Status

[![Build Status](https://travis-ci.org/roboticslab-uc3m/gait.svg?branch=master)](https://travis-ci.org/roboticslab-uc3m/gait)

[![Coverage Status](https://coveralls.io/repos/github/roboticslab-uc3m/gait/badge.svg)](https://coveralls.io/github/roboticslab-uc3m/gait)

[![Issues](https://img.shields.io/github/issues/roboticslab-uc3m/gait.svg?label=Issues)](https://github.com/roboticslab-uc3m/gait/issues)

