cmake_minimum_required(VERSION 3.4...3.18)
project(car)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -Wall -mcpu=native -flto -pthread -g")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(INCLUDE_PATH "/usr/local/include")
set(LIB_PATH "/usr/local/lib")
find_package(OpenCV QUIET COMPONENTS core videoio highgui imgproc imgcodecs ml video)

message(STATUS "OpenCV found (${OpenCV_CONFIG_PATH})，${OpenCV_LIBS}")

#---------------------------------------------------------------------
#       [ Include、Lib  : Path ] ==> [ glib opencv ]
#---------------------------------------------------------------------
set(PROJECT_INCLUDE_PATH "${PROJECT_SOURCE_DIR}/include")
include_directories(${PROJECT_INCLUDE_PATH})

find_package(PkgConfig)
pkg_search_module(GLIB REQUIRED glib-2.0)
include_directories(${GLIB_INCLUDE_DIRS})

find_package( OpenCV REQUIRED )    
include_directories( ${OpenCV_INCLUDE_DIRS} )
link_directories(${OpenCV_INCLUDE_DIRS})

# find ppnc
pkg_search_module(PPNC REQUIRED ppnc)
include_directories(${PPNC_INCLUDE_DIRS})
link_directories(${PPNC_LIBRARY_DIRS})

# find onnx
pkg_search_module(ONNX REQUIRED onnx)
include_directories(${ONNX_INCLUDE_DIRS})
link_directories(${ONNX_LIBRARY_DIRS})

set(COMMON_LIB_DIR "${PROJECT_SOURCE_DIR}/../lib/")
link_directories(${COMMON_LIB_DIR})

# set(COMMON_SOURCES
# 		)
set(DETECTION "detection" )
add_executable(${DETECTION} ${COMMON_SOURCES} src/detection.cpp)

target_link_libraries(${DETECTION} ${PPNC_LIBRARIES})
target_link_libraries(${DETECTION} ${ONNX_LIBRARIES})
target_link_libraries(${DETECTION} ${OpenCV_LIBS})
target_link_libraries(${DETECTION} pthread)
target_link_libraries(${DETECTION} shared_mem)
