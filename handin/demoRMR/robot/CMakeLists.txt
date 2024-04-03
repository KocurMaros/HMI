# Add your source files
set(SOURCES
	CKobuki.cpp
	robot.cpp
	rplidar.cpp
)

set(HEADERS
	CKobuki.h
	robot_global.h
	robot.h
	rplidar.h
	szevent.h
)

add_library(robot ${SOURCES} ${HEADERS})

# Add headers
target_include_directories(robot PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
	${OpenCV_INCLUDE_DIRS}
)

# Link OpenCV libraries
target_link_libraries(robot ${OpenCV_LIBRARIES})

# Specify output directory
set_target_properties(robot PROPERTIES
	LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../bin
)
