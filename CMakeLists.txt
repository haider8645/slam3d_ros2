cmake_minimum_required(VERSION 3.5)
project(slam3d_ros2)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(slam3d 2.5 REQUIRED)

add_executable(pointcloud_mapper
	src/pointcloud_mapper.cpp
)

target_link_libraries(pointcloud_mapper PRIVATE
	rclcpp::rclcpp
	${std_msgs_TARGETS}
	slam3d::solver-g2o
	slam3d::graph-boost
	slam3d::sensor-pcl
)

install(
	TARGETS pointcloud_mapper
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
