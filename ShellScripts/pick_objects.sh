#!/bin/sh
export TURTLEBOT_GAZEBO_WORLD_FILE=$(rospack find world)/room.world
export TURTLEBOT_GAZEBO_MAP_FILE=$(rospack find world)/room_map.yaml

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "rosrun pick_objects pick_objects" &
sleep 5
xterm -e "echo 'Testing pick objects with 2 markers'; rostopic pub -1 /visualization_marker visualization_msgs/Marker -f /home/amay/catkin_ws/src/pick_objects/src/test_marker1.yaml; sleep 5; rostopic pub -1 /visualization_marker visualization_msgs/Marker -f /home/amay/catkin_ws/src/pick_objects/src/test_marker2.yaml"
