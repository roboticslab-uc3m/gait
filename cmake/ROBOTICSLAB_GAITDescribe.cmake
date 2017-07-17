# CMake installation path.
if(WIN32 AND NOT CYGWIN)
    set(GAIT_CMAKE_DESTINATION CMake)
else()
    set(GAIT_CMAKE_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gait)
endif()

# Add all targets to the build-tree export set.
export(TARGETS Gait
       NAMESPACE ROBOTICSLAB::
       FILE ${PROJECT_NAME}Targets.cmake)

# Store the package in the user registry.
export(PACKAGE ${PROJECT_NAME})

# Generate simple <pkg>Config.cmake file.
file(WRITE ${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake
     "include(\${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}Targets.cmake)")

# Install GAITConfig.cmake
install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        DESTINATION ${GAIT_CMAKE_DESTINATION})

# Install the export set for use with the install-tree.
install(EXPORT ${PROJECT_NAME}
        NAMESPACE ROBOTICSLAB::
        FILE ${PROJECT_NAME}Targets.cmake
        DESTINATION ${GAIT_CMAKE_DESTINATION})
