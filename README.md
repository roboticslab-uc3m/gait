# gait
c++ Humanoid gait generation library.

Use as system lib or as source code.

For automatic (system preferred) Add this to Cmakelists.txt:

	\#gait  
	\#Change directory here if Gait instalation is not found  
	set(GAIT_LOCATION /usr/local)  
	message(STATUS "Looking for local Gait in: [${GAIT_LOCATION}].")  
	\# TODO:force to refresh find_library every run  
	find_library(gait_LIBRARY NAMES gait libgait PATHS "${GAIT_LOCATION}/lib/gait/")  
	if(${gait_LIBRARY} STREQUAL "gait_LIBRARY-NOTFOUND")  
	    \#Try to use the source code if not installed  
	    message(STATUS "Gait not installed. Trying to compile gait from sources")  
	    add_subdirectory(${PROJECT_SOURCE_DIR}/lib/gait)  
	    set(GAIT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/lib/gait/src ${GAIT_INCLUDE_DIR})  
	    set(gait_LIBRARY "gait")  
	else()  
	    \#Try to use the local lib if installed  
	    message(STATUS "Gait library found. Using local files")  
	    FIND_PATH(GAIT_INCLUDE_DIR Gait.h /usr/local/include/gait/)  
	    message(STATUS "Local Gait files detected: [${gait_LIBRARY}].")  
	    message(STATUS "Local include directories: [${GAIT_INCLUDE_DIR}].")  
	endif()  
	INCLUDE_DIRECTORIES(${GAIT_INCLUDE_DIR})  
