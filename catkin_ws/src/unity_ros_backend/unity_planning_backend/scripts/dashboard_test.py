#!/usr/bin/python3

from enum import auto
import rospy

from backend_support.URDashboard import URDashboard
from backend_support.RobotiqGripperController import Robotiq2FController

installation_name = 'jerry_remote.installation'
program_name      = 'jerry_ext.urp'

if __name__ == '__main__':
    rospy.init_node('UR_ROS_dashboard_node', anonymous=True)

    dashboard = URDashboard()
    
    # dashboard.load_installation(installation_name)
    # dashboard.cold_boot()
    dashboard.load_program(program_name)
    dashboard.start_program()
    
    rospy.spin()
    
    # input('Press any key to continue...')
    
    # controller = Robotiq2FController(autoconnect=True)
    
    # input('Press any key to continue...')

    # controller.activate_gripper()
    # controller.autoclose(force=255)

    
    # controller.reset_gripper()
    # controller.activate_gripper()
    
    # input('Press any key to continue...')
    
    # while not rospy.is_shutdown():
        # controller.print_status()
        # input('Press any key to continue...')
        
        # controller.autoclose(force=25)
        # input('Press any key to continue...')
        
        # controller.autoopen(force=25)
        # input('Press any key to continue...')
        