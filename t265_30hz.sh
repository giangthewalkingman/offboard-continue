#!/bin/bash
roslaunch realsense2_camera rs_t265.launch 
rosrun topic_tools throttle messages /camera/odom/sample 30.0 /odom
