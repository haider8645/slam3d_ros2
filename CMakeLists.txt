cmake_minimum_required(VERSION 3.16.3)
project(slam3d_ros2)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(slam3d 2.5 REQUIRED)

# Needed by pcl_conversions, but not properly exported in foxy
find_package(PCL 1.7 REQUIRED COMPONENTS common)

add_executable(pointcloud_mapper
	src/pointcloud_mapper.cpp
)

target_include_directories(pointcloud_mapper PUBLIC
	${pcl_conversions_INCLUDE_DIRS}
)

target_link_libraries(pointcloud_mapper
	rclcpp::rclcpp
	tf2_ros::tf2_ros
	tf2_eigen::tf2_eigen
	${sensor_msgs_TARGETS}
	${std_srvs_TARGETS}
	${PCL_COMMON_LIBRARIES}
	slam3d::solver-g2o
	slam3d::graph-boost
	slam3d::sensor-pcl
)

install(
	TARGETS pointcloud_mapper
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
