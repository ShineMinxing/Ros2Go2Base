from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ip_camera',
            executable='ip_camera_node',
            name='ip_camera_node',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='usb_camera',
            executable='usb_camera_node',
            name='usb_camera_node',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='usb_camera',
            executable='usb_camera_node',
            name='usb_camera_node2',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='yolo_obb',
            executable='yolo_obb_node',
            name='yolo_obb_node',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='yolo_obb',
            executable='yolo_obb_node',
            name='yolo_obb_node2',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='yolo_obb',
            executable='yolo_obb_node',
            name='yolo_obb_node3',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='rviz2', 
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.expanduser('~/ros2_ws/LeggedRobot/src/Ros2Go2Base/other/SMXFE_odm.rviz')]
        ),
    ])
