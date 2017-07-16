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
