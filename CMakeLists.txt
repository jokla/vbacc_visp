cmake_minimum_required(VERSION 2.8.3)
project(vbacc)



find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  visp_bridge
)

find_package(VISP REQUIRED)

# Make sure Boost.Filesystem v3 is used.
add_definitions(-DBOOST_FILESYSTEM_VERSION=3)

# Add ViSP definitions.
add_definitions(${VISP_DEFINITIONS})


catkin_package(
  
#  INCLUDE_DIRS
#  include
  
  LIBRARIES
  vbacc
  
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf

  DEPENDS 
  VISP
  visp_bridge
)

include_directories(include)

include_directories(SYSTEM
  ${VISP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


# Declare a cpp library
add_library(vbacc
  include/vbacc.h
  src/vbacc.cpp
)


target_link_libraries(vbacc
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
)

## Declare a cpp executable
add_executable(vbacc_node src/main.cpp)


  target_link_libraries(vbacc_node
    ${resource_retriever_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    vbacc
    )


