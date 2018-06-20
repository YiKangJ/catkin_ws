#!/bin/bash

gnome-terminal -x bash -c "roslaunch kuca kuca_simple.launch"
  
sleep 3
  
gnome-terminal -x bash -c "roslaunch kuca_simple_moveit_config move_group.launch" 
  
sleep 5
  
gnome-terminal -x bash -c "rosrun rviz rviz -d `rospack find kuca`/config/attached_object.rviz"

