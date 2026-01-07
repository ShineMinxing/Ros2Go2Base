import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 【关键修改】修正了你之前的路径拼写错误 (src 和 Ros2Go2Base 之间缺了 /)
    slam_params_file = '/home/smx/WorkSpace/GDS_LeggedRobot/src/Ros2Go2Base/other/slam_params.yaml'

    # 使用 Node 直接启动，而不是调用外部终端
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file]
    )

    return LaunchDescription([
        slam_toolbox_node
    ])