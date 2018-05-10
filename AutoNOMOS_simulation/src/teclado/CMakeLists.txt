cmake_minimum_required(VERSION 2.8.3)
project(teclado)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Qt4 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES example
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs image_transport
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include( ${QT_USE_FILE} )
add_definitions( ${QT_DEFINITIONS} )

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

include_directories(${QT_INCLUDES} include include/teclado)

## Declare a C++ executable
add_executable(teclado 
				src/main.cpp 
				src/teclado.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(teclado 
					${QT_LIBRARIES}
					${catkin_LIBRARIES})

