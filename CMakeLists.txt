cmake_minimum_required(VERSION 2.8.3)

project(rb1_500e_study)

set(CMAKE_CXX_COMPILER g++)
set(CMAKE_CXX_STANDARD 11)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
)

# RBDL and EIGEN setting
find_package(RBDL REQUIRED)
find_package(Eigen3 3.0.0 REQUIRED)
find_package(gazebo REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS ${BOOST_BUILD_COMPONENTS})

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS} 
  ${RBDL_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
SET(${PROJECT_NAME}_HEADERS
  
  src/main.h 

)
LIST(REMOVE_DUPLICATES ${PROJECT_NAME}_HEADERS)

SET(${PROJECT_NAME}_SOURCES
  src/main.cpp
)
ADD_LIBRARY(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${RBDL_LIBRARY} ${RBDL_URDFReader_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

catkin_package(
  LIBRARIES rb1_500e_study
  CATKIN_DEPENDS gazebo_ros roscpp rospy std_msgs
  DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")