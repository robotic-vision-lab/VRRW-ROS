#!/usr/bin/env python3

import rospy

from robotiq_ros_wrapper.msg import Robotiq2FInput, Robotiq2FOutput
from rospy.exceptions import ROSException

from robotiq_ros_controller.RobotiqMisc import *

class Robotiq2FCommander():
    def __init__(self):
        self.current_status:Robotiq2FInput = None
        
        self.register()
        
    def register(self, timeout:int = 10):
        try:
            status_topic = rospy.get_param('/robotiq_2f_controller/status_topic', default='/Robotiq2F/gripper_status')
            command_topic = rospy.get_param('/robotiq_2f_controller/command_topic', default='/Robotiq2F/command_interface')
            rospy.wait_for_message(status_topic, Robotiq2FInput, timeout=timeout)
            self.status_monitor = rospy.Subscriber(status_topic, Robotiq2FInput, callback=self.status_monitor_callback)
            self.command_publisher = rospy.Publisher(command_topic, Robotiq2FOutput, queue_size=1)
            rbt_log_warn(f'Waiting for {5} seconds so publishers and subscribers register correctly')
            rospy.sleep(5)
            self.reset()
            rbt_log_success(f'Robotiq Commander registered successfully and ready to plan')
        except ROSException as error:
            rbt_log_error(error)
            
    def reset(self):
        pass
    
    def deactivate(self):
        pass
        
    def activate(self):
        pass
        
    def status_monitor_callback(self, msg):
        self.current_status = msg
        
    def print_status(self) -> None:
        pass
    
    def is_moving(self):
        return (True if self.current_status.object_status == 0 else False)
    
    def is_holding(self):
        pass
    
    def goto_jaw_angle(self):
        pass
    
    def goto_opening(self):
        pass
    
    def goto_raw(self):
        pass
    
    def auto_open(self):
        pass
    
    def auto_close(self):
        pass
    
    def set_force_N(self):
        pass
    
    def set_force_raw(self):
        pass