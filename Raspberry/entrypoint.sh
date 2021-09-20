#!/bin/bash

echo "entrypoint here...."

source /opt/ros/melodic/setup.bash
source /usr/src/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11310
export ROS_HOSTNAME=localhost

# lock update lock-file, so we can't update without override
# lockfile /tmp/balena/updates.lock

# launch ros program
# roslaunch my_roslaunch  myprogram.launch
