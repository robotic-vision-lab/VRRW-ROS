#!/usr/bin/env python3

import rospy

from rvl_robotiq_controller.RobotiqController import Robotiq2FController

rospy.init_node('robotiq_testing_node', anonymous = True)

r = Robotiq2FController(stroke = 85, initialize = True)
r.open_gripper(17.23)