from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        # -----------------------------
        # Node auf Raspberry Pi (remote)
        # -----------------------------
        ExecuteProcess(
            cmd=[
                'ssh',
                'ubuntu@10.42.0.1',  
                'source /home/pi/my_ws/install/setup.bash && ros2 run pi_package pi_node'
            ],
            shell=True
        ),

        # -----------------------------
        # Drei Action-Server Nodes lokal
        # -----------------------------
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

        # -----------------------------
        # Eine lokale Node im selben Paket wie die Action-Server
        # -----------------------------
        Node(
            package='muri_dev',
            executable='muri_action_handler',
            name='muri_action_handler',
            output='screen'
        ),

        # -----------------------------
        # Eine Node lokal, aber in einem anderen Paket
        # -----------------------------
        Node(
            package='vision_proccessing',
            executable='image_data_processing',
            name='image_data_processing',
            output='screen'
        ),
    ])
