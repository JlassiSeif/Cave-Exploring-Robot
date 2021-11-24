cmake_minimum_required(VERSION 3.0.2)
project(robot_wheels)

find_package(catkin REQUIRED
	roscpp
	visualization_msgs
	pcl_conversions
    pcl_ros
	)

catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS roscpp
                   pcl_conversions
                   pcl_ros
  )
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(plot scripts/plot.cpp)
target_link_libraries(plot ${catkin_LIBRARIES})
add_executable(lidar scripts/lidar.cpp)
target_link_libraries(lidar ${catkin_LIBRARIES})
add_executable(marker scripts/marker.cpp)
target_link_libraries(marker ${catkin_LIBRARIES})
