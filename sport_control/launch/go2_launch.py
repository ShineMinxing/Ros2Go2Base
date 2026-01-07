from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    
    # --- 第一组：立即启动的节点 ---
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='log'
    )

    control_message_node = Node(
        package='control_message',
        executable='control_message_node',
        name='control_message_node',
        output='log'
    )

    sport_control_node = Node(
        package='sport_control',
        executable='sport_control_node',
        name='sport_control_node',
        output='screen'
    )

    # --- 第二组：延时 1.0 秒启动的节点 ---
    fusion_estimator_node = Node(
        package='fusion_estimator',
        executable='fusion_estimator_node',
        name='fusion_estimator_node',
        output='screen'
    )

    # --- 第三组：延时 2.0 秒启动的节点 ---
    dds_rostopic_node = Node(
        package='dds_rostopic',
        executable='dds_rostopic_node',
        name='dds_rostopic_node',
        output='log'
    )

    message_handle_node = Node(
        package='message_handle',
        executable='message_handle_node',
        name='message_handle_node',
        output='screen'
    )

    return LaunchDescription([
        # 1. 立即启动
        joy_node,
        control_message_node,
        sport_control_node,

        # 2. 延时 1s 启动 Fusion
        TimerAction(
            period=1.0,
            actions=[fusion_estimator_node]
        ),

        # 3. 延时 2s 启动 DDS 和 Message Handle
        TimerAction(
            period=2.0,
            actions=[dds_rostopic_node, message_handle_node]
        )
    ])