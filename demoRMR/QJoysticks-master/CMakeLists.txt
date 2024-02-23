# Add your source files
set(SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/src/QJoysticks.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/QJoysticks/SDL_Joysticks.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/QJoysticks/VirtualJoystick.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/QJoysticks/Android_Joystick.cpp
)

# Add headers
set(HEADERS
	${CMAKE_CURRENT_SOURCE_DIR}/src/QJoysticks.h
	${CMAKE_CURRENT_SOURCE_DIR}/src/QJoysticks/JoysticksCommon.h
	${CMAKE_CURRENT_SOURCE_DIR}/src/QJoysticks/SDL_Joysticks.h
	${CMAKE_CURRENT_SOURCE_DIR}/src/QJoysticks/VirtualJoystick.h
	${CMAKE_CURRENT_SOURCE_DIR}/src/QJoysticks/Android_Joystick.h
)

# Add resources
set(RESOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/etc/resources/qjoysticks-res.qrc
)

# Add executable
add_library(qjoysticks ${SOURCES} ${HEADERS} ${RESOURCES})

# Include directories
target_include_directories(
	qjoysticks
PUBLIC
	${SDL_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Link libraries
target_link_libraries(
	qjoysticks
PUBLIC
	Qt5::Core
	Qt5::Gui
	Qt5::Widgets
	${SDL_LIBRARIES}
)
