FROM balenalib/raspberrypi3-64-ubuntu:bionic-build
LABEL MAINTAINER="Frederik Mazur Andersen <fm@mmmi.sdu.dk>"

RUN [ "cross-build-start" ]

# set timezone data
ENV TZ=Europe/Copenhagen
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Update and install
RUN install_packages procmail nano git ssh openssh-client make cmake build-essential socat gcc g++ python3-dev net-tools inetutils-ping python3 python3-pip python3-wheel libgeos-dev daemontools lsb-release
#RUN apt-get upgrade -y

# install ros. Standalone step to avoid having to reinstall on all changes
# RUN install_ros.bash
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update
RUN  apt-get install -y ros-melodic-ros-base ros-melodic-smach

# Setup application folder
ENV APP=/usr/src/app
ENV WS=/usr/home/eit_ws

# Enable UDEV for balena
ENV UDEV=on
WORKDIR $APP

## copy source code
COPY eit_playground $WS/src/eit_playground
RUN install_packages  ros-melodic-mavros-msgs
WORKDIR $WS
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"
RUN "ls" 
# RUN "catkin_make"

# Run entrypoint script
COPY Raspberry/entrypoint.sh $APP/
RUN chmod +x $APP/entrypoint.sh

CMD ["/usr/src/app/entrypoint.sh"]

RUN [ "cross-build-end" ]