#!/bin/sh
xterm -e " source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find my_robot)/worlds/My_Robot_World.world" &
sleep 6
xterm -e " source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find turtlebot_gazebo)/maps/my_map.yaml" &
sleep 6
xterm -e " source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 6
xterm -e " source devel/setup.bash; rosrun add_markers add_markers_testing"
