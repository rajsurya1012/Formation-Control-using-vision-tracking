# Formation Control using Vision Tracking

The objective is to move a set of holonomic robots while adhering to geometric constraints between them by using an RGB camera.
Authors: Raj Surya, Santo Santhosh


## Setup Required to run

1. ROS 1 Noetic
2. Open CV
3. Python 3
4. Gazebo
5. Teleop Twist keyboard package

## Steps to run

1. Run `catkin_make` in your workspace.
2. Run `roslaunch multi_robot main.launch `, this should open the environment in gazebo.
3. In a new terminal, run `rosrun multi_robot image.py`.
4. In a new terminal run, `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot1/cmd_vel`.
5. By using the keys, you should be able to control the robot and see the other robots following its motion.
