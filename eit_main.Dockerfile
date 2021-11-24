#Base Image :
#FROM ubuntu 20.04 
FROM px4io/px4-dev-ros-noetic
#ROS Installation (Version :Noetic) || http://wiki.ros.org/noetic/Installation/Ubuntu :
 	# sudo exec sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'	
 	# sudo apt install curl
 	# curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
	# sudo apt install ros-noetic-desktop-full
 	# source /opt/ros/noetic/setup.bash
 	# echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
	# source ~/.bashrc	
	# sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
	# sudo rosdep init
	# rosdep update

# # Configure ROS Environment || http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment :
# 	source /opt/ros/noetic/setup.bash
# 	mkdir -p ~/catkin_ws/src
# 	cd ~/catkin_ws/
# 	catkin_make
# 	source devel/setup.bash

# # Get the PX4 SITL simulation up and running PDF pg 40 || Appendix D:
RUN sudo apt update && sudo apt install astyle build-essential ccache clang clang-tidy cmake cppcheck doxygen \
	file g++ gcc gdb git lcov make ninja-build \
	python3 python3-dev python3-pip python3-setuptools python3-wheel  \
	rsync shellcheck unzip xsltproc zip libeigen3-dev libopencv-dev libroscpp-dev \
	protobuf-compiler ninja-build gstreamer1.0-plugins-bad gstreamer1.0-plugins-base \
	gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly libgstreamer-plugins-base1.0-dev libgstrtspserver-1.0-dev xvfb -y

RUN pip install --user argparse cerberus empy jinja2 numpy packaging pandas psutil pygments pyros-genmsg pyserial pyulog pyyaml setuptools six toml wheel rosdep
RUN pip3 install --user --upgrade empy jinja2 numpy packaging pyros-genmsg toml pyyaml pymavlink

ENV USER=user
WORKDIR /home/user
RUN	git clone --branch v1.12.1 --depth 1 https://github.com/PX4/PX4-Autopilot

WORKDIR /home/user/PX4-Autopilot

RUN git submodule update --init --recursive
RUN DONT_RUN=1 make px4_sitl_default gazebo

# # #Build the PX4-Autopilot SITL firmware and run the Gazebo environment:

# # #Run the PX4 SITL environment (¡¡¡THIS LEADS TO THE COMMAND LINE OF THE PX4!!!):
# RUN	cd ~/PX4-Autopilot
# RUN make px4_sitl_default gazebo

# # #Source the PX4-Autopilot firmware:
RUN /bin/bash -c "echo alias sitl=\'source /home/user/PX4-Autopilot/Tools/setup_gazebo.bash  /home/user/PX4-Autopilot /home/user/PX4-Autopilot/build/px4_sitl_default\' >> /home/user/.bashrc"
# RUN /bin/bash -c "echo alias t=\'cd..\' >> /home/user/.bashrc"

RUN	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/user/PX4-Autopilot
RUN	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/user/PX4-Autopilot/Tools/sitl_gazebo
# # 	#(OPTIONAL)--> Add the comands to the .bashrc, idk how (in terminal)

# #Experts in Teams Playground working example for
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod 755 install_geographiclib_datasets.sh
RUN sudo ./install_geographiclib_datasets.sh

WORKDIR /home/user
RUN mkdir -p eit_ws/src
WORKDIR /home/user/eit_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
	catkin_init_workspace &&\
	# cd .. && \
	# catkin build &&\
	# cd src && \
	git clone --depth 1 https://gitlab.com/sdu-uas-eit/eit_playground.git  &&\
	cd ..&& \
	catkin build"
RUN /bin/bash -c "echo \"alias seit-ws='source /home/user/eit_ws/devel/setup.bash  && source /home/user/eit_ws/src/eit_playground/setup_gazebo.bash'\" >> /home/user/.bashrc"
RUN /bin/bash -c "echo \"alias peit-ws='cd /home/user/eit_ws/ && seit-ws'\" >> /home/user/.bashrc"
RUN /bin/bash -c "echo \"alias beit-ws='peit-ws && catkin build'\" >> /home/user/.bashrc"

RUN /bin/bash -c "echo \"alias l_eitpg='seit-ws && roslaunch eit_playground posix.launch vehicle:=sdu_drone_mono_cam_downward env:=aruco_test_3'\" >> /home/user/.bashrc"
RUN /bin/bash -c "echo \"alias lh_eitpg='seit-ws && roslaunch eit_playground posix.launch vehicle:=sdu_drone_mono_cam_downward env:=aruco_test_3 gui:=false'\" >> /home/user/.bashrc"

RUN /bin/bash -c "echo \"alias l_mavros='seit-ws && roslaunch mavros px4.launch fcu_url:=\"udp://:14540@127.0.0.1:14557\"'\" >> /home/user/.bashrc"
RUN /bin/bash -c "echo \"alias l_offbnode='seit-ws && rosrun eit_playground offb_node'\" >> /home/user/.bashrc"

# Build SDU drone
RUN sed -i -e 's/~/home\/user/g' /home/user/eit_ws/src/eit_playground/setup_posix.bash
RUN /bin/bash -c "source /home/user/eit_ws/src/eit_playground/setup_posix.bash"
RUN /bin/bash -c "cd /home/user/eit_ws/ && source /home/user/eit_ws/devel/setup.sh  && source /home/user/eit_ws/src/eit_playground/setup_gazebo.bash && catkin build"
RUN apt-get install vim -y

RUN export PX4_HOME_LAT=55.471650
RUN export PX4_HOME_LON=10.328990
RUN export PX4_HOME_ALT=0

RUN apt update && apt-get install ros-noetic-rqt -y

RUN apt update && apt-get install python3-opencv -y

