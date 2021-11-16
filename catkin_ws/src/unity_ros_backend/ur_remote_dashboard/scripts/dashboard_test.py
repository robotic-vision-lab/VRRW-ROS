#!/usr/bin/python3

from enum import auto
from math import radians, degrees
from moveit_commander.move_group import MoveGroupCommander
import rospy

from ur_remote_dashboard.URDashboard import URDashboard

import moveit_commander as mc

installation_name = 'jerry_remote.installation'
program_name      = 'jerry_ext.urp'

sequence = 1

if __name__ == '__main__':
    rospy.init_node('UR_ROS_dashboard_node', anonymous=True)

    dashboard:URDashboard = URDashboard()
    
    if sequence == 1:
        dashboard.load_installation(installation_name)
        dashboard.cold_boot()
        dashboard.load_program(program_name)
        dashboard.start_program()
    elif sequence == 2:
        dashboard.stop_program()
    elif sequence == 3:
        dashboard.stop_program()
        dashboard.start_program()
    elif sequence == 4:
        dashboard.log('test message')
        dashboard.warn('test message')
        dashboard.log_success('test message')
        dashboard.log_error('test_message')
    elif sequence == 5:
        dashboard.get_robot_mode()
        dashboard.get_safety_mode()