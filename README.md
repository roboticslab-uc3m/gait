# gait
c++ biped gait generation library.

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
