from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory("agrisarthi_gps_nav"), "config", "hardware.yaml")
    return LaunchDescription([
        Node(package="agrisarthi_gps_nav", executable="gps_waypoint_navigator",
             name="gps_waypoint_navigator", parameters=[config], output="screen"),
        Node(package="agrisarthi_gps_nav", executable="mission_sampling_manager",
             name="mission_sampling_manager", parameters=[config], output="screen"),
    ])
