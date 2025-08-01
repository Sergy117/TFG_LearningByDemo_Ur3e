This is the ROS Package for moving the robot, real or simulated

1. Launch conection with robot. Robot stopped but powered on
```sh
    roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=172.22.24.161
```
if it is the simulated robot, use instead
```sh
roslaunch ur3e_moveit_config demo.launch
```
2. Launch the planifier, after unlocked robot motors and initiated external control in UR3e
```sh
roslaunch ur3e_moveit_config moveit_planning_execution.launch
```
3. Launch Rviz for visualization (Optional)
```sh
roslaunch ur3e_moveit_config moveit_rviz.launch
```
4. Launch ROS package that moves the robot
```sh
rosrun ur3e_control ur3e_pick_and_place.py
```