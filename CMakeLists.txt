cmake_minimum_required(VERSION 3.25)
project(LidarCarTracking)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  laser_filters
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  rosconsole
)

find_package( OpenCV REQUIRED )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  header
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidarproject
#  CATKIN_DEPENDS laser_geometry pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

add_executable(laser_to_merged_pc source/laser_to_merged_pc.cpp)
target_link_libraries(laser_to_merged_pc ${catkin_LIBRARIES})

add_executable(car_tracking source/car_tracking.cpp)
target_link_libraries(car_tracking ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(picars_logic source/picars_logic.cpp)
target_link_libraries(picars_logic ${catkin_LIBRARIES})
