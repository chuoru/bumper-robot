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

# Internal library 


class BumperControl(object):
    """!
    @brief This class provide bumper control 
    """
    # ==========================================================================
    # PUBLIC FUNCTION
    # ==========================================================================
    def __init__(self) -> None:
        pass
    
    def run(self):
        """! Start ros node
        """
        rospy.spin()
