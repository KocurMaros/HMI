# Add your source files
set(SOURCES
	QLed.cpp
)

set(HEADERS
	QLed.h
)

add_library(qled ${SOURCES} ${HEADERS})

# Add headers
target_include_directories(qled PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
	${OpenCV_INCLUDE_DIRS}
)

# Link OpenCV libraries
target_link_libraries(
	qled
PUBLIC
	Qt::Core
	Qt::Widgets
	Qt::Gui
)
