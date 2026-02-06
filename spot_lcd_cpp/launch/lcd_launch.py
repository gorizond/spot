from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spot_lcd_cpp',
            executable='spot_lcd_cpp_lcd_node',
            name='lcd_node',
            output='screen',
            parameters=[]
        )
    ])