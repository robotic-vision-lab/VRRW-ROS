#!/usr/bin/python3

import rospy

from backend_support.URDashboard import URDashboard, IOFunctions, IOStates, DigitalIOMapping

if __name__ == '__main__':
    rospy.init_node('UR_ROS_dashboard_node', anonymous=True)

    dashboard = URDashboard()
    
    rospy.spin()