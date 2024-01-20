cmake_minimum_required(VERSION 3.5.1)
project(PaddleLiteSample)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "-g")

find_package(PaddleLite QUIET)
find_package(OpenCV QUIET COMPONENTS core videoio highgui imgproc imgcodecs ml video)

message(STATUS "PaddleLite found (${PADDLELITE_LIBRARY})")
message(STATUS "OpenCV found (${OpenCV_CONFIG_PATH})，${OpenCV_LIBS}")

set(COMMON_SOURCES
		src/core/capture.cpp
		src/core/display.cpp
		src/core/multi_channel_camera.cpp
		src/utils/multi_channel_camera_util.cpp
		)
		
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")
if (PADDLELITE_FOUND)
	include_directories(${PADDLELITE_INCLUDE_DIR})
	include_directories(${OpenCV_INCLUDE_DIRS})
 
  link_directories(../lib)

	set(DETECTION "detection" )
	add_executable(${DETECTION} ${COMMON_SOURCES} src/detection.cpp)
	target_link_libraries(${DETECTION}
    ${PADDLELITE_LIBRARY}
	  ${OpenCV_LIBS}
    -lshared_mem) 
else ()
	message(FATAL_ERROR "Error !!!! : PADDLELITE NOT FOUND")
endif ()
