#!/bin/sh
export ROBOT_INITIAL_POSE="-x 1.702 -y 1.492 -z 0"
xterm  -hold -e  "source /home/workspace/catkin_ws/devel/setup.bash;roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/worlds/kiril.world" & 
sleep 5
xterm -e "source /home/workspace/catkin_ws/devel/setup.bash ; roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "source /home/workspace/catkin_ws/devel/setup.bash ; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
