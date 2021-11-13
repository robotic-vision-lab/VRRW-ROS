#!/usr/bin/python3

from enum import auto
from math import radians, degrees
from moveit_commander.move_group import MoveGroupCommander
import rospy

from ur_remote_dashboard.URDashboard import URDashboard

import moveit_commander as mc

installation_name = 'jerry_remote.installation'
program_name      = 'jerry_ext.urp'

if __name__ == '__main__':
    rospy.init_node('UR_ROS_dashboard_node', anonymous=True)

    dashboard = URDashboard()