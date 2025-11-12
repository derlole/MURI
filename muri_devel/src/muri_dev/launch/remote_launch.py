from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Node camera_read_out on raspi 
        ExecuteProcess(
            cmd=[
                'sshpass', '-p', 'ipek2023',
                'ssh', '-o', 'StrictHostKeyChecking=no',
                'ubuntu@10.42.0.1',
                'source /home/ubuntu/muri_team/muri_robot/install/setup.bash && ros2 run vision_proccessing camera_readout'
            ],
            shell=True
        ),
    ])
