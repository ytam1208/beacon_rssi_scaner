cmake_minimum_required(VERSION 2.8.3)
project(Beacon_Scan_)

# add_compile_options(-std=c++11)
set(CMAKE_VERBOSE_MAKEFILE yes)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-O0 -DNDEBUG -std=c++11")

find_package(catkin REQUIRED  COMPONENTS
    roscpp
    std_msgs
    std_srvs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include/
)

add_executable(${PROJECT_NAME}_node src/scan_one_beacon.cpp src/SimpleKalmanFilter.cpp)


add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  -lbluetooth
)


