#!/usr/bin/python3

import rospy
from rospy.service import ServiceException

from ur_dashboard_msgs.srv import Load
from std_srvs.srv import Trigger

dashboard_serv = '/ur_hardware_interface/dashboard/'
dashboard_load = dashboard_serv + 'load_program'
dashboard_play = dashboard_serv + 'play'

prog_name = 'jerry_ext.urp'

if __name__ == '__main__':
    rospy.init_node('wrist_gripper_controller', anonymous=True)
    
    print(f'Waiting for {dashboard_load} service to come up...')
    rospy.wait_for_service(dashboard_load)
    print(f'Waiting for {dashboard_play} service to come up...')
    rospy.wait_for_service(dashboard_play)
    
    prog_loader = None
    prog_trigger = None
    response = None
    
    try:
        prog_loader = rospy.ServiceProxy(dashboard_load, Load)
        prog_trigger = rospy.ServiceProxy(dashboard_play, Trigger)
        response = prog_loader(prog_name)
        print(f'Loading {prog_name} success = {str(response.success).upper()}')
        response = prog_trigger()
        print(f'Starting {prog_name} success = {str(response.success).upper()}')
    except ServiceException as e:
        print(f'Something went wrong!')
        print(e)
    
    rospy.spin()