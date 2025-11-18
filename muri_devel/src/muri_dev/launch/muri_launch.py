from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='muri_dev',
            executable='muri_action_server_drive',
            name='muri_action_server_drive',
            output='screen'
        ),
        Node(
            package='muri_dev',
            executable='muri_action_server_turn',
            name='muri_action_server_turn',
            output='screen'
        ),
        Node(
            package='muri_dev',
            executable='muri_action_server_init',
            name='muri_action_server_init',
            output='screen'
        ),
        # Node(
        #     package='muri_dev',
        #     executable='muri_action_handler',
        #     name='muri_action_handler',
        #     output='screen'
        # ),
        Node(
            package='vision_proccessing',
            executable='image_data_processing',
            name='image_data_processing',
            output='screen'
        ),
        Node(            
            package='vision_proccessing',
            executable='camera_readout',
            name='camera_readout',
            output='screen'
        ),
    ])
