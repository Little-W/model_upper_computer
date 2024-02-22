cmake_minimum_required(VERSION 3.0.2)
project(car)

set(CMAKE_CXX_FLAGS "-g")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV found (${OpenCV_INCLUDE_DIRS})")
SET(BASE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../)
# set(CXX_FLAGS_DEBUG "-O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
link_directories(
  ${BASE_SOURCE_DIR}/lib
)


add_executable(main_node
	src/main.cpp
	src/process.cpp
	src/pid.cpp
	src/reader.cpp
	src/comm.cpp
)

target_link_libraries(main_node
	${OpenCV_LIBS}
	-lyaml-cpp
	-lserial
	-lshared_mem
	-flto
)
