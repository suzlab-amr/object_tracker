cmake_minimum_required(VERSION 3.0.2)
project(object_tracker)

find_package(catkin REQUIRED 
  roscpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  hungarian_algorithm_lib
  kalman_filter_lib
  object_msgs
)

find_package(cmake_modules REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(jsoncpp REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(state_tracker src/state_tracker.cpp)
target_compile_options(state_tracker PUBLIC -O3 -Wall)

add_library(object_tracker src/object_tracker.cpp)
target_link_libraries(object_tracker ${catkin_LIBRARIES} state_tracker jsoncpp_lib)
target_compile_options(object_tracker PUBLIC -O3 -Wall)

add_executable(object_tracker_node src/object_tracker_node.cpp)
target_link_libraries(object_tracker_node object_tracker)
target_compile_options(object_tracker_node PUBLIC -O3 -Wall)

install(TARGETS object_tracker_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
