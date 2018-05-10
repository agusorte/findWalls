cmake_minimum_required(VERSION 3.0)
project(find_walls)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package( OpenCV REQUIRED )

find_package(PCL 1.8 REQUIRED )

include_directories(${PCL_INCLUDE_DIRS}
					${OPENCV_INCLUDE} 
					${EIGEN3_INCLUDE_DIR})


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SRCS main.cpp findWalls.cpp )



add_executable(find_walls  ${SRCS})

target_link_libraries(find_walls   ${PCL_LIBRARIES} ${OpenCV_LIBS})