#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2022 chuoru <chuoru@github.com>
#
# Distributed under terms of the MIT license.
# Standard library


# External library 
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

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

        self.control_timer = rospy.Timer(rospy.Duration(0.5), self._control_callback)

        self._left_bumper_value = False

        self._right_bumper_value = False
    
    def run(self) -> None:
        """! Start ros node
        """
        rospy.spin()
    
    # ==========================================================================
    # PRIVATE FUNCTION
    # ==========================================================================
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
        