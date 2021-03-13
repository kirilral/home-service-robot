#!/bin/sh
export ROBOT_INITIAL_POSE="-x 1.8 -y 1.8 -z 0 -R 0 -P 0 -Y 1.50718"
xterm  -hold -e  "source /home/workspace/catkin_ws/devel/setup.bash;roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/worlds/kiril.world" & 
sleep 5
xterm -e "source /home/workspace/catkin_ws/devel/setup.bash ; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/map/map.yaml" &
sleep 5
xterm -e "source /home/workspace/catkin_ws/devel/setup.bash ; roslaunch add_markers rviz.launch rviz_config_file_path:=/home/workspace/catkin_ws/config/rvizconfig.rviz" &
sleep 25
xterm -e "source /home/workspace/catkin_ws/devel/setup.bash ; rosrun add_markers add_markers" & 
sleep 25
xterm -e "source /home/workspace/catkin_ws/devel/setup.bash ; rosrun pick_objects pick_objects" 
