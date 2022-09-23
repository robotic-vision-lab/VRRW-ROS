#!/usr/bin/env python

import rospy
import sys
import moveit_commander

from copy import deepcopy
from math import pi, tau, radians, degrees
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface

from rvl_unity_backend.srv import *
from std_msgs.msg import Header

joint_names = ["shoulder_pan_joint",
               "shoulder_lift_joint",
               "elbow_joint",
               "wrist_1_joint",
               "wrist_2_joint",
               "wrist_3_joint"]

def main(args):
    rospy.init_node('unity_moveit_backend_planner', anonymous=True)
    moveit_commander.roscpp_initialize(args)

    robot = RobotCommander()
    scene = PlanningSceneInterface()
    arm = MoveGroupCommander("arm")
    gripper = MoveGroupCommander("gripper")

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)