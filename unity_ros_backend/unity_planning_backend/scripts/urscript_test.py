#!/usr/bin/python3

import rospy

from backend_support.URDashboard import URDashboard, IOFunctions, IOStates, DigitalIOMapping

from std_msgs.msg import String

inst_name = 'jerry_remote.installation' # installation file, setting external_control IP to my IP
prog_name = 'jerry_ext.urp'             # program file, basically loading external_control

if __name__ == '__main__':
    rospy.init_node('UR_ROS_dashboard_node', anonymous=True)

    dashboard = URDashboard()
    dashboard.load_installation(inst_name)  # will cause dashboard disconnection and power down
    dashboard.cold_boot()                   # restart the arm to normal operation
    dashboard.load_program(prog_name)       # load external_control
    dashboard.start_program()               # start external control
    
    rospy.spin()