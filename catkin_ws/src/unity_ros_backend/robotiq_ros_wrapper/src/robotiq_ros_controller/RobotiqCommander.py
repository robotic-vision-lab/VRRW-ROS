#!/usr/bin/env python3

from numpy.lib.type_check import common_type
import rospy

from robotiq_ros_wrapper.msg import Robotiq2FInput, Robotiq2FOutput
from rospy.exceptions import ROSException

from robotiq_ros_controller.RobotiqMisc import *

class Robotiq2FCommander():
    def __init__(self):
        # load parameter from server
        self.default_force = rospy.get_param('robotiq_2f_controller/default_force', default=100)
        self.default_speed = rospy.get_param('robotiq_2f_controller/default_speed', default=100)
        self.calibrated_lower = rospy.get_param('robotiq_2f_controller/calibrated_lower_limit', default=0)
        self.calibrated_upper = rospy.get_param('robotiq_2f_controller/calibrated_upper_limit', default=255)
        self.stroke = rospy.get_param('robotiq_2f_controller/stroke', default=85)
        
        # keep track of current status
        self.current_status = None
        
        # register correct topics for monitoring and control
        self.register()
        
    def calibrate(self):
        raise NotImplementedError
            
    def register(self, timeout = 10):
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
            rbt_log_error('Unable to assign Robotiq Commander to gripper')
            rbt_log_error(error)
            
    def reset(self, wait = 2):
        rbt_log_warn(f'Gripper reset called. Waiting for {wait} seconds')
        self.deactivate()
        self.activate()
        rospy.sleep(wait)
    
    def deactivate(self):
        command = Robotiq2FOutput()
        command.activate = 0
        self.compensated_publish(command)
        while self.current_status.activated != 0:
            pass
        
    def activate(self):
        command = Robotiq2FOutput()
        command.activate = 1
        self.compensated_publish(command)
        while self.current_status.gripper_status != 3 or self.current_status.activated != 1:
            pass
        
    def status_monitor_callback(self, msg):
        self.current_status = msg
        
    def print_status(self) -> None:
        pass
    
    def is_moving(self):
        return (self.current_status.object_status == 0)
    
    def is_holding(self):
        return (self.current_status.object_status == 1 or self.current_status.object_status == 2)
    
    def set_speed(self, value, unit='raw'):
        acceptable_units = ['mm/s', 'in/s', 'raw']
        if unit not in acceptable_units:
            rbt_log_error(f'Unable to set : {unit} is not a valid unit')
            rbt_log_error(f'Acceptable units are {acceptable_units}')
            return False
        
    def set_force(self, value, unit = 'raw'):
        acceptable_units = ['N', 'lbf', 'raw']
        if unit not in acceptable_units:
            rbt_log_error(f'Unable to set : {unit} is not a valid unit')
            rbt_log_error(f'Acceptable units are {acceptable_units}')
            return False
        
    def go_to_position(self, position, speed = None, force = None, blocking = False, unit = 'raw'):
        acceptable_units = ['mm', 'in', 'rad', 'raw']
        if unit not in acceptable_units:
            rbt_log_error(f'Unable to set : {unit} is not a valid unit')
            rbt_log_error(f'Acceptable units are {acceptable_units}')
            return False
        
    def auto_open(self, speed = None, force = None, blocking = False):
        command = Robotiq2FOutput()
        command.activate = 1
        command.goto = 1
        command.position = self.calibrated_lower
        command.speed = (self.default_speed if speed is None else speed)
        command.force = (self.default_force if force is None else force)
        self.compensated_publish(command)
        rospy.sleep(0.1)
        while self.is_moving():
            rospy.sleep(0.05)
        return (self.current_status.current_position == self.calibrated_lower)
    
    def auto_close(self, speed = None, force = None, blocking = False):
        command = Robotiq2FOutput()
        command.activate = 1
        command.goto = 1
        command.position = self.calibrated_upper
        command.speed = (self.default_speed if speed is None else speed)
        command.force = (self.default_force if force is None else force)
        self.compensated_publish(command)
        rospy.sleep(0.1)
        while self.is_moving():
            rospy.sleep(0.05)
        return (self.current_status.current_position == self.calibrated_upper)
    
    def compensated_publish(self, command, lag = 0.1):
        self.command_publisher.publish(command)
        rospy.sleep(lag)
