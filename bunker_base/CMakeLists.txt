cmake_minimum_required(VERSION 2.8.3)
project(bunker_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS    
    roslaunch 
    roscpp 
    sensor_msgs
    std_msgs
    geometry_msgs
    tf2 
    tf2_ros
    ugv_sdk
    bunker_msgs
)
    
# find_package(Boost REQUIRED COMPONENTS chrono)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bunker_messenger
  CATKIN_DEPENDS ugv_sdk bunker_msgs
#   DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(bunker_messenger STATIC src/bunker_messenger.cpp)
target_link_libraries(bunker_messenger ${catkin_LIBRARIES})
add_dependencies(bunker_messenger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(bunker_base_node src/bunker_base_node.cpp)
target_link_libraries(bunker_base_node bunker_messenger ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS bunker_messenger bunker_base_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
