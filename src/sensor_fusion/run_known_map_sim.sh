#!/bin/bash
 
my_pid=$$
echo "My process ID is $my_pid"

echo "Launching roscore..."
roscore &
pid=$!
sleep 5s

echo "Launching Gazebo..."
roslaunch sensor_fusion sensor_fusion_gazebo.launch &
pid="$pid $!"

sleep 5s

#echo "Launching navigation stack..."
#roslaunch nav_bundle nav_bundle.launch &

echo "Launching map server..."
rosrun map_server map_server ~/ROS/maps/map.yaml &
pid="$pid $!"

sleep 3s

echo "Launching controller..."
roslaunch pioneer_ros pioneer_controller_spin_recover.launch &
pid="$pid $!"

echo "Launching Rviz..."
roslaunch pioneer_description frontier_map.launch rviz_name:=pioneer &
pid="$pid $!"

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
