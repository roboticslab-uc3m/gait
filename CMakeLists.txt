cmake_minimum_required(VERSION 2.8)

##functions
function(echo_all_cmake_variable_values)
  message(STATUS "")
  get_cmake_property(vs VARIABLES)
  foreach(v ${vs})
    message(STATUS "${v}='${${v}}'")
  endforeach(v)
  message(STATUS "")
endfunction()

project(gait)


# source files for gait
aux_source_directory(src GAIT_SRCS)
#INCLUDE_DIRECTORIES(src)

FILE(GLOB_RECURSE GAIT_INCLUDE_LIST src/*.h src/*.hpp)


#SET(GAIT_SRCS spgait)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${GAIT_SRCS} ${GAIT_INCLUDE_LIST})


#Install Library targets if top level project
if("${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")
    #install target
    message(STATUS "Configuring gait as main project")
    install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME})
    install(FILES ${GAIT_INCLUDE_LIST}
        DESTINATION include/${PROJECT_NAME})

    # uninstall target
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)
    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

#Set Library variables if subdirectory (called from add_subdirectory(${PROJECT_SOURCE_DIR}/lib/gait) )
else("${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")
    message(STATUS "Configuring gait as sub-project")
    set(GAIT_INCLUDE_DIR ${CURRENT_SOURCE_DIR}/src ${GAIT_INCLUDE_DIR})
    set(GAIT_LIBRARY ${CURRENT_BINARY_DIR})
    set(GAIT_LIBRARIES ${GAIT_LIBRARY} )
    set(GAIT_INCLUDE_DIRS ${GAIT_INCLUDE_DIR} )

endif()

#orocos kdl
#  message(STATUS "kdl libraries")
#  add_subdirectory(external/orocos_kinematics_dynamics/orocos_kdl/)
#  set(KDL_INCLUDE_DIR external/orocos_kinematics_dynamics/orocos_kdl/src/ ${KDL_INCLUDE_DIR})
#    include_directories(${KDL_INCLUDE_DIR})

#link with orocos
#target_link_libraries(gait orocos-kdl)


#echo_all_cmake_variable_values()
