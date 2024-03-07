## Wall following robot
## Author: Nishant Awdeshkumar Pandey
## About
The turtlebot follows the left wall while avoiding collision and obstacles (narrowly), while also trying to get as close to the left wall as possible. 
The bug algorithm is used for showcasing the desired behavior and avoiding obstacles.

## Dependencies:

1. ROS2 (Humble was used)
2. Turtlebot 3 
3. Gazebo and RViz

## Steps to run package:

1. ```mkdir ros2_ws```
2. ```cd ros2_ws```
3. ```mkdir src```
4. ```cd src```
5. copy and paste the package in ros2_ws/src or any workspace you see fit.
6. ```cd ..```
7. ```colcon build --symlink-install```
8. ```source ~/ros2_ws/install/setup.bash```
9. ```source /opt/ros/humble/setup.bash```
10. ```source /usr/share/gazebo/setup.sh```
11. ```export TURTLEBOT3_MODEL=burger```
12. ```ros2 launch obstacle_avoidance turtlebot3_house.launch.py```.
13. In the new terminal after repeating commands 8-10, to teleoperate the robot ```ros2 run turtlebot3_teleop teleop_keyboard```.
14. In the new terminal after repeating commands 8-10 for simulating the wall following robot ```ros2 run obstacle_avoidance turtlebot3_bug_algorithm```.

Note:
If there are any errors encountered while running these commands source the desired workspaces and directories needed by your system properly.


