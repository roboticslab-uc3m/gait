# gait
c++ biped gait generation library.

Basic lines to add/modify in CMake:

```cmake
find_package(GAIT REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${GAIT_INCLUDE_DIRS})

link_directories(${GAIT_LINK_DIRS})
target_link_libraries(${KEYWORD} ${GAIT_LIBRARIES})
```

