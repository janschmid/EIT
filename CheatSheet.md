# Prepare QGroundControl
Install QGroundControl
Setup the UDP port like described here: https://docs.px4.io/master/en/test_and_ci/docker.html#qgroundcontrol
# Docker-help
Build and run the container docker-compose build && docker-compose run gazebo bash
# EIT playground
- \<project-name\> is eit-ws
-  s\<project-name\>: Source the catkin workspace of the project and the Firmware (eit_ws and eit_playground)
-  p\<project-name\>: Source both projects and go to the catkin workspace of the project
-  b\<project-name\>: Build, Source both projects and go to the catkin workspace of the project
  
## Launch
Px4 SITL simulation environment, keep in mind to run seit-ws in advanced 

`seit-ws && roslaunch eit_playground posix.launch vehicle:=sdu_drone`, vehicle is optional

`seit-ws && roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"`

`seit-ws && rosrun eit_playground offb_node`

optional vehicle parameters are:
- sdu_drone_mono_cam
- sdu_drone_stereo_cam
- sdu_drone_depth_cam
- sdu_drone_lidar
- sdu_drone_sonar
for optitrack, use vehicle:=sdu_drone env:=optitrack

more can be found on EIT_E21_UAS_v1.1 p58ff
