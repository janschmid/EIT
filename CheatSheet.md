# Prepare QGroundControl
Install QGroundControl
Setup the UDP port like described here: https://docs.px4.io/master/en/test_and_ci/docker.html#qgroundcontrol
# Docker-help
Build and run the container `docker-compose -f simulation-docker-compose.yml build && docker-compose -f simulation-docker-compose.yml run gazebo bash`
# EIT playground
- \<project-name\> is eit-ws
-  s\<project-name\>: Source the catkin workspace of the project and the Firmware (eit_ws and eit_playground)
-  p\<project-name\>: Source both projects and go to the catkin workspace of the project
-  b\<project-name\>: Build, Source both projects and go to the catkin workspace of the project
  
## Launch
Px4 SITL simulation environment, keep in mind to run seit-ws in advanced 
- l_eitpg or headless with lh_eigpg
    - `seit-ws && roslaunch eit_playground posix.launch vehicle:=sdu_drone_mono_cam_downward env:=demo`, vehicle is optional
- l_mavros
    - `seit-ws && roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"`
- l_offbnode
    - `seit-ws && roslaunch eit_playground eit.launch`

optional vehicle parameters are:
- sdu_drone_mono_cam
- sdu_drone_stereo_cam
- sdu_drone_depth_cam
- sdu_drone_lidar
- sdu_drone_sonar
for optitrack, use vehicle:=sdu_drone env:=optitrack

more can be found on EIT_E21_UAS_v1.1 p58ff

## Run Gazebo in headless mode
If you have issues running gazebo gui in docker, run it instead in headless mode and launch the client on your local machine. Proceed as following:
- Install gazebo `curl -sSL http://get.gazebosim.org | sh`
- Create symlink from: eit_playground/models to ~/.gazebo/models
- Launch gazebo client with `gzclient` and rosnode in container

# Balena
- Install [Balena CLI](https://github.com/balena-io/balena-cli/releases)
- Setup local lan connection (share wifi with lan), IPV4 Method: Automatic (DHCP) 
- Connect Raspberry Pi to Laptop via lan
- Goto folder EIT
- Comment out "Cross-build" sections in Raspberry/Dockerfile
- Run: balena push a1870e9.local

# Raspberry pi
- Auto connect to hidden network: https://blog.khmersite.net/2020/12/auto-connect-to-hidden-wifi-with-networkmanger/