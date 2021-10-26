#!/usr/bin/python3

import rospy

from backend_support.URDashboard import URDashboard #, IOFunctions, IOStates, DigitalIOMapping

installation_name = 'jerry_remote.installation'
program_name      = 'jerry_ext.urp'

if __name__ == '__main__':
    rospy.init_node('UR_ROS_dashboard_node', anonymous=True)

    dashboard = URDashboard()
    
    # dashboard.load_installation(installation_name)
    # dashboard.cold_boot()
    # dashboard.load_program(program_name)
    # dashboard.start_program()
    
    # input("Press Enter to continue...")

    # dashboard.stop_program()
    # dashboard.power_off_arm()

    rospy.spin()