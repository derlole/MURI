from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='muri_dev',
            executable='muri_action_server_drive',
            name='muri_action_server_drive',
            output='log'
        ),
        Node(
            package='muri_dev',
            executable='muri_action_server_turn',
            name='muri_action_server_turn',
            output='log'
        ),
        Node(
            package='muri_dev',
            executable='muri_action_server_init',
            name='muri_action_server_init',
            output='log'
        ),
        Node(
            package='muri_dev',
            executable='muri_action_server_follow',
            name='muri_action_server_follow',
            output='log'
        ),
        Node(
            package='muri_dev',
            executable='muri_action_handler',
            name='muri_action_handler',
            output='log'
        ),
        Node(
            package='vision_proccessing',
            executable='image_data_processing',
            name='image_data_processing',
            output='log'
        ),
        Node(            
            package='vision_proccessing',
            executable='camera_readout',
            name='camera_readout',
            output='log'
        ),
    ])
