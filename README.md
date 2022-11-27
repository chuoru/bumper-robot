# Bumper robot
ROS packages for robot control with bumper sensor reading

## Configuration
Go to file nitra_bringup.launch and modify these parameters
```
<arg name="robot_port" default="/dev/ttyUSB0" />
<arg name="bumper_port" default="/dev/ttyUSB1" />
<arg name="bumper_baud" default="9600" />
```

## How to build 
```
catkin_make
```

## How to run 
```
source devel/setup.bash
roslaunch nitrabot_launch nitrabot_bringup.launch
```

## How to run with docker
```
cd nitrabot_dockerfiles
docker-compose up -d 
```