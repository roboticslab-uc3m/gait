project(gait)


# source files for gait
aux_source_directory(src GAIT_SRCS)
#INCLUDE_DIRECTORIES(src)

FILE(GLOB_RECURSE GAIT_INCLUDE_LIST src/*.h src/*.hpp)


#SET(GAIT_SRCS spgait)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${GAIT_SRCS} ${GAIT_INCLUDE_LIST})


#orocos kdl
#  message(STATUS "kdl libraries")
#  add_subdirectory(external/orocos_kinematics_dynamics/orocos_kdl/)
#  set(KDL_INCLUDE_DIR external/orocos_kinematics_dynamics/orocos_kdl/src/ ${KDL_INCLUDE_DIR})
#    include_directories(${KDL_INCLUDE_DIR})

#link with orocos
#target_link_libraries(gait orocos-kdl)
