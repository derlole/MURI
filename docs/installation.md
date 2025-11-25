# Installation Steps

## System requirements
Make sure that your System meets following requirements:
1. Is a Turtlebot3 which brings itselt the funtctions to give data about his odometry on the topic /odom, accepts velocity Commands with a type Twist on /cmd_vel and use this commands internally for his own Movement

2. You Have Access to  a Camera on your System.

3. You have globally installed python3 with the public pip (or other python package manager) packages installed: cv2

4. You have installed ROS2 on the Turtlebot.

5. You have acces to install further packages via pip.

## Steps
1. Clone the Github repository into a folder of your choice

2. change in following directory an execute colcon build 
cd MURI/muri_devel
colcon build

3. source the ros workspace 
source install/setup.bash

4. change in following directory and install the python package
cd MURI/muri_logics
pip install -e .

5. If your System needs different requirements e.g Camera Calibaration or any regulator parameters go into MURI/muri_logics and change the needed parameters

6. 
