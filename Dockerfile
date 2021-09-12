# To import image from .tar file:
# cat eit_ros_px4_gzb_qgc_08_09.tar | docker import - eit_ros_px4_gzb_qgc_08_09

FROM eit_ros_px4_gzb_qgc_08_09
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN sudo apt install ros-noetic-mavros ros-noetic-mavros-extras -y
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod 755 install_geographiclib_datasets.sh
RUN sudo ./install_geographiclib_datasets.sh

# ^ This part above works, what comes below doesn't:
# Error: cannot find catkin_init_workspace and also catkin_make

RUN source /opt/ros/noetic/setup.bash
RUN mkdir -p ~/eit_ws/src
RUN cd ~/eit_ws/src/
RUN catkin_init_workspace

RUN cd ~/eit_ws/
RUN catkin_make

RUN cd ~/eit_ws/src/
RUN git clone https://gitlab.com/sdu-uas-eit/eit_playground.git

RUN cd ~/eit_ws/
RUN catkin_make

RUN source ~/eit_ws/src/eit_playground/setup_gazebo.bash
RUN cd ~
RUN echo "alias seit-ws=\"source ~/eit_ws/devel/setup.bash && source ~/eit_ws/src/eit_playground/setup_gazebo.bash\"\
		alias peit-ws=\"cd ~/eit_ws/ && seit-ws\" \
		alias beit-ws=\"peit-ws && catkin build\"" >> .bashrc

RUN beit-ws
