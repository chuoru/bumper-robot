#!/bin/sh
set -e
basic_dep="git \
           curl \
           nano \
           vim \
           python3-catkin-tools \
           python3-pip \
		   libserial-dev
	  "

ros_dep="ros-noetic-rosserial \
          "

apt-get update
apt-get upgrade -y
apt-get install -y $basic_dep
DEBIAN_FRONTEND=noninteractive apt-get install -y $ros_dep
apt-get autoremove -y
apt-get clean -y
update-alternatives --install /usr/bin/python python /usr/bin/python3.8 1
python --version
python -m pip install pyserial


mkdir -p /root/catkin_ws/src
cd /root/catkin_ws/src || exit 1