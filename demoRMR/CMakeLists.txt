cmake_minimum_required(VERSION 3.22)
project(DemoRMR LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)

set(SDL_INCLUDE_DIRS /usr/include/SDL2)
set(SDL_LIBRARIES /usr/lib/x86_64-linux-gnu/libSDL2.so)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)

add_subdirectory(robot)
add_subdirectory(QJoysticks-master)
add_subdirectory(QLed)
add_definitions(-DMAP_PATH="${CMAKE_CURRENT_SOURCE_DIR}/priestor.txt")
add_definitions(-std=c++17)

message("SDL_LIBRARIES: ${SDL_LIBRARIES}")
message("SDL_INCLUDE_DIRS: ${SDL_INCLUDE_DIRS}")
message("OpenCV_LIBS: ${OpenCV_LIBS}")
message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
set(HEADERS
	BodyProgressBars.h
	ControllButtons.h
	PositionTracker.h
	HelpWindow.h
	MapLoader.h
	mainwindow.h
	stylesheeteditor.h
	ControllButtons.h
	HelpWindow.h
	BodyProgressBars.h
    object_detection.h
	PidController.h
	RobotTrajectoryController.h
	FloodPlanner.h
)

set(SOURCES
	BodyProgressBars.cpp
	ControllButtons.cpp
	PositionTracker.cpp
	HelpWindow.cpp
	MapLoader.cpp
	main.cpp
	mainwindow.cpp
	stylesheeteditor.cpp
	ControllButtons.cpp
	HelpWindow.cpp
	BodyProgressBars.cpp
    object_detection.cpp
	PidController.cpp
	RobotTrajectoryController.cpp
	FloodPlanner.cpp
)

set(UI
	mainwindow.ui
	stylesheeteditor.ui
	HelpWindow.ui
)

set(QRC
	demoRMR.qrc
)

add_executable(rmr
	${HEADERS}
	${SOURCES}
	${UI}
	${QRC}
)

target_link_libraries(
	rmr
PUBLIC
	Qt::Core
	Qt::Widgets
	${OpenCV_LIBS}
	robot
	qjoysticks
	qled
)
