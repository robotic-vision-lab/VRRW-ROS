#!/usr/bin/python3

import rospy

from backend_support.URDashboard import URDashboard, IOFunctions, IOStates, DigitalIOMapping

from std_msgs.msg import String

prog_name = 'jerry_ext.urp'

if __name__ == '__main__':
    rospy.init_node('UR_ROS_dashboard_node', anonymous=True)

    dashboard = URDashboard()
    
    rospy.spin()