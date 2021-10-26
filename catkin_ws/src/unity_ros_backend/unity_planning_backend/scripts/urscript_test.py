#!/usr/bin/python3

import rospy

from backend_support.URDashboard import URDashboard #, IOFunctions, IOStates, DigitalIOMapping

installation_name = 'jerry_remote.installation'
program_name      = 'jerry_ext.urp'

if __name__ == '__main__':
    rospy.init_node('UR_ROS_dashboard_node', anonymous=True)

    dashboard = URDashboard()
    dashboard.release_brakes()