from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

   
    slam3d_ros2_params = PathJoinSubstitution(
        [
            FindPackageShare("slam3d_ros2"),
            "config",
            "parameters.yaml",
        ]
    )

    slam3d_ros2_node = Node(
        package="slam3d_ros2",
        executable="pointcloud_mapper",
        output="screen",
        parameters=[slam3d_ros2_params],
        #arguments=['--ros-args', '--log-level', 'debug']
    )

    return LaunchDescription([SetParameter(name='use_sim_time', value=True),slam3d_ros2_node])
