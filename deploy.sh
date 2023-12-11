#!/bin/sh

REMOTE="ryan@minimcgeev4.lan"

rsync -av --delete /home/ryan/Documents/workspace/catkin_ws/src/ $REMOTE:/home/ryan/catkin_ws/src/
ssh $REMOTE "source /opt/ros/noetic/setup.bash && cd /home/ryan/catkin_ws && catkin_make" # && source devel/setup.bash && pkill robot && ROS_MASTER_URI=http://minimcgeev4.local:11311 rosrun minimcgeev4 robot
