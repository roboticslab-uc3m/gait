# gait
c++ biped gait generation library.

## Use as an external library

Basic lines to add/modify in CMake:

```cmake
find_package(GAIT REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${GAIT_INCLUDE_DIRS})

link_directories(${GAIT_LINK_DIRS})
target_link_libraries(${KEYWORD} ${GAIT_LIBRARIES})
```

For CMake find_package(GAIT REQUIRED), you may also be interested in adding the following to your .bashrc or .profile:

```bash
export GAIT_DIR=/home/teo/repos/gait/build
```

Change the path according to your folder structure.

## Use as part of a project

In [kinematics-dynamics](https://github.com/roboticslab-uc3m/kinematics-dynamics) we added these lines to `kinematics-dynamics/libraries/CMakeFiles.txt`:

```cmake
set(GAIT_PART_OF_PROJECT TRUE)
add_subdirectory(gait)
```

And then we added a hardcoded a `kinematics-dynamics/cmake/FindGAIT.cmake` for it to be found at compilation:

```cmake
IF (NOT GAIT_FOUND)

  SET(GAIT_LIBRARIES "Gait")
  SET(GAIT_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/libraries/gait/src/")
  SET(GAIT_LINK_DIRS "${CMAKE_SOURCE_DIR}/build/lib")
  SET(GAIT_DEFINES "")
  SET(GAIT_MODULE_PATH "./gait/cmake")

  SET (GAIT_FOUND TRUE)
ENDIF (NOT GAIT_FOUND)
```

Another example of use can be found at [gaitcontrol](https://github.com/roboticslab-uc3m/gaitcontrol), especially at `gaitcontrol/cmake` folder.

## Status

[![Build Status (Linux/OSX)](https://img.shields.io/travis/roboticslab-uc3m/gait/master.svg?label=Build Status (Linux/OSX) )](https://travis-ci.org/roboticslab-uc3m/gait)

[![Coverage Status](https://coveralls.io/repos/roboticslab-uc3m/gait/badge.svg)](https://coveralls.io/r/roboticslab-uc3m/gait)

[![Issues](https://img.shields.io/github/issues/roboticslab-uc3m/gait.svg?label=Issues)](https://github.com/roboticslab-uc3m/gait/issues)

