from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Node camera_read_out on raspi 
        # ExecuteProcess(
        #     cmd=[
        #         'sshpass', '-p', 'ipek2023',
        #         'ssh', '-o', 'StrictHostKeyChecking=no',
        #         'ubuntu@10.42.0.1',
        #         'source /home/ubuntu/muri_team/muri_robot/install/setup.bash && ros2 run vision_proccessing camera_readout'
        #     ],
        #     shell=True
        # ),
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
        Node(
            package='muri_dev',
            executable='muri_action_handler',
            name='muri_action_handler',
            output='screen'
        ),
        Node(
            package='vision_proccessing',
            executable='image_data_processing',
            name='image_data_processing',
            output='screen'
        ),
    ])
