# Robotics_Studio_2
Pick and Place using UR3

Clone these files into a src folder in a workspace and build.

Dependencies-
1. Universal_Robots_ROS_Driver- https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
2. Moveit- https://github.com/moveit/moveit
3. OnRobot Gripper- https://github.com/Osaka-University-Harada-Laboratory/onrobot

Command sequence-
 
roslaunch rosbridge_server rosbridge_websocket.launch
 
roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=192.168.1.100 kinematics_config:="${HOME}/my_robot_calibration.yaml"
 
roslaunch ur3_rg2_moveit_config move_group.launch
 
roslaunch ur3_rg2_moveit_config moveit_rviz.launch config:=true
 
roslaunch onrobot_rg_control bringup.launch ip:=192.168.1.1
 
python3 ur3_control.py

Working- 


https://github.com/UdhayROB/Robotics_Studio_2/assets/143361662/c08eea34-985d-4666-b72b-cd8b1ed06c8a


https://github.com/UdhayROB/Robotics_Studio_2/assets/143361662/9a42e940-36eb-4eb3-bda3-3f0bdbd0687e

