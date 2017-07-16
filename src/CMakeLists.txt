#-- Compiler issues, see target_compile_options() command below and https://github.com/roboticslab-uc3m/gait/issues/6
#--------------------------------------------------------------------------------------------------------------------
#-- Not doing because requires cmake 3.1 [source of original code](https://crascit.com/2015/03/28/enabling-cxx11-in-cmake/):
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#-- Not doing because requires gcc/g++ 4.7 and higher. [source of original code](http://stackoverflow.com/questions/20826135/passing-std-c11-to-cmakelists):
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#-- Lower than gcc/g++, as travis uses 4.6.3 [source of original code](https://github.com/travis-ci/travis-ci/issues/4634):
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

add_library(Gait Gait.cpp
                 Gait.h
                 GaitLipm.cpp
                 GaitLipm.h
                 GaitPatterns.cpp
                 GaitPatterns.hpp
                 GaitSupportPoligon.cpp
                 GaitSupportPoligon.h
                 Oscillator.cpp
                 Oscillator.hpp
                 tools.cpp
                 tools.h
                 ZeroMomentPoint.cpp
                 ZeroMomentPoint.hpp)

set_property(TARGET Gait PROPERTY PUBLIC_HEADER Gait.h
                                                GaitLipm.h
                                                #GaitPatterns.hpp
                                                GaitSupportPoligon.h
                                                #Oscillator.hpp
                                                tools.h
                                                #ZeroMomentPoint.hpp
                                                )

target_compile_options(Gait PRIVATE "-std=c++0x")

target_include_directories(Gait PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                       $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(TARGETS Gait
        EXPORT GAIT_EXPORT
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
