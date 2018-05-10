cmake_minimum_required(VERSION 2.8.3)
project(skycam)

if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=gnu++0x)
endif()

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  cv_bridge
  sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES skycam
#  CATKIN_DEPENDS roscpp cv_bridge sensor_msgs image_transport
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

include_directories(include include/skycam)

## Declare a C++ executable
add_executable(skycam 
				src/skycam_node.cpp 
				src/skycam_nodelet.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(skycam 
				${catkin_LIBRARIES}
				${OpenCV_LIBS})

