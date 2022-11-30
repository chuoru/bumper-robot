#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2022 chuoru <chuoru@github.com>
#
# Distributed under terms of the MIT license.
# Standard library
import math

# External library 
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from math import pi

# Internal library 


class BumperControl(object):
    """!
    @brief This class provide bumper control 
    """
    # ==========================================================================
    # PUBLIC FUNCTION
    # ==========================================================================
    def __init__(self) -> None:
        """! Class constructor
        """
        # Initialization of ROS related variable
        rospy.init_node("bumper_control_node", anonymous=True)

        self.left_bumper_subscriber = rospy.Subscriber(
            "/left_bumper", Bool, self._left_bumper_callback
        )

        self.left_bumper_subscriber = rospy.Subscriber(
            "/right_bumper", Bool, self._right_bumper_callback
        )

        self.velocity_publisher = rospy.Publisher(
            "/cmd_vel", Twist, queue_size=1)
        
        self._linear_velocity = 0.1 # m/s

        self._angular_velocity = 0.1 # rad/s

        self._sampling_time = 0.5

        self.control_timer = rospy.Timer(rospy.Duration(self._sampling_time), self._control_callback)

        self._left_bumper_value = False

        self._right_bumper_value = False

        self._mode = "normal"

        self._index = 0

        self._initialize_system_parameter()

        self._generate_backing_motion()

    def run(self) -> None:
        """! Start ros node
        """
        rospy.spin()
    
    # ==========================================================================
    # PRIVATE FUNCTION
    # ==========================================================================
    def _initialize_system_parameter(self):
        """! Calculate system parameter such as backing distance. 
        """
        self._a = 0

        self._b = 0

        self._x = 0

        self._alpha = pi/2

        self._d = 1

        self._alpha_1 = pi/4

        self._d_1 = 2

    def _left_bumper_callback(self, msg: Bool) -> None:
        """! Callback function for the left bumper rostopic.
        @param[in] msg: odometry data in pose stamped form
        """
        self._left_bumper_value = msg.data

    def _right_bumper_callback(self, msg: Bool) -> None:
        """! Callback function for the right bumper rostopic.
        @param[in] msg: odometry data in pose stamped form
        """
        self._right_bumper_value = msg.data

    def _control_callback(self, event) -> None:
        """! Callback function for the control timer.
        @param[in] msg: odometry data in pose stamped form
        """
        self._validate_mode()

        if self._mode == "normal":
            self._move_straight(self._linear_velocity)

        elif self._mode in ["collide_right", "collide_left", "collide"]:
            motion = self._backing_motion[self._mode]

            if self._index >= len(motion): 
                self._model = "normal"
                return

            command = self._backing_motion[self._mode][self._index]

            if command[0] != 0:
                self._move_straight(command[0])

            else: 
                self._turn(command[1])

            self._index += 1

        else:
            raise NotImplemented

    def _generate_backing_motion(self):
        """! Validate current mode of robot 
        @param[in] mode: mode of collision
        """
        self._backing_motion = {}

        duration = round((self._d_1 / self._linear_velocity) / self._sampling_time)
        
        self._backing_motion["collide_left"] = [(-self._linear_velocity, 0) for _ in range(duration)]

        duration = round((self._alpha_1 / self._angular_velocity) / self._sampling_time)

        self._backing_motion["collide_left"].extend([(0, self._angular_velocity) for _ in range(duration)])

        duration = round((self._d_1 / self._linear_velocity) / self._sampling_time)
        
        self._backing_motion["collide_right"] = [(-self._linear_velocity, 0) for _ in range(duration)]

        duration = round((self._alpha_1 / self._angular_velocity) / self._sampling_time)

        self._backing_motion["collide_right"].extend([(0, -self._angular_velocity) for _ in range(duration)])

        duration = round((self._d / self._linear_velocity) / self._sampling_time)
        
        self._backing_motion["collide"] = [(-self._linear_velocity, 0) for _ in range(duration)]

        duration = round((self._alpha / self._angular_velocity) / self._sampling_time)

        self._backing_motion["collide"].extend([(0, self._angular_velocity) for _ in range(duration)])

    def _validate_mode(self) -> None:
        """! Validate current mode of robot 
        """
        if self._mode != "normal":
            return            

        if self._left_bumper_value and self._right_bumper_value:
            self._index = 0
            self._mode = "collide"

        elif self._left_bumper_value:
            self._index = 0
            self._mode = "collide_left"

        elif self._right_bumper_value:
            self._index = 0
            self._mode = "collide_right"

	#rospy.loginfo(self._mode);
	
    def _move_straight(self, velocity: float) -> None:
        """! Move robot straight
        @param[in] velocity: linear velocity of robot 
        """
        velocity_msg = Twist()

        velocity_msg.linear.x = velocity

        velocity_msg.angular.z = 0

        self.velocity_publisher.publish(velocity_msg) 

    def _turn(self, velocity: float) -> None:
        """! Turn robot 
        @param[in] velocity: angular velocity of robot 
        """
        velocity_msg = Twist()

        velocity_msg.linear.x = 0

        velocity_msg.angular.z = velocity

        self.velocity_publisher.publish(velocity_msg) 
    
    def _stop(self) -> None:
        """! Stop robot
        """
        velocity_msg = Twist()

        velocity_msg.linear.x = 0

        velocity_msg.angular.z = 0

        self.velocity_publisher.publish(velocity_msg)
