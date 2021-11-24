#!/usr/bin/env python3

from numpy.lib.type_check import common_type
import rospy

from robotiq_ros_wrapper.msg import Robotiq2FInput, Robotiq2FOutput
from rospy.exceptions import ROSException

from robotiq_ros_controller.RobotiqMisc import *
from robotiq_ros_controller.Robotiq2FSupport import *

class Robotiq2FCommander():
    def __init__(self, initial_calib = False):
        # load parameter from server
        self.default_force      = rospy.get_param('robotiq_2f_controller/raw_force', default=100)
        self.default_speed      = rospy.get_param('robotiq_2f_controller/raw_speed', default=100)
        self.limit              = tuple(rospy.get_param('robotiq_2f_controller/raw_range', default=[0, 255]))
        self.stroke             = rospy.get_param('robotiq_2f_controller/stroke', default=85)
        self.publisher_delay    = rospy.get_param('robotiq_2f_controller/command_delay', default=0.1)
        self.finger_thickness   = rospy.get_param('robotiq_2f_controller/finger_thickness', default=7.67)
        
        # ease of reading
        self.calibrated_lower = self.limit[0]
        self.calibrated_upper = self.limit[1]
        
        # relevant topics
        self.status_topic = rospy.get_param('/robotiq_2f_controller/status_topic', default='/Robotiq2F/gripper_status')
        self.command_topic = rospy.get_param('/robotiq_2f_controller/command_topic', default='/Robotiq2F/command_interface')

        # keep track of current status
        self.current_status = None
        
        # converters
        self.conv_p = robotiq_distance(self.limit, self.stroke)
        self.conv_f = robotiq_force(self.stroke)
        self.conv_s = robotiq_speed(self.stroke)
        
        self.conv_p.raw = 0
        self.conv_f.raw = self.default_force
        self.conv_s.raw = self.default_speed
        
        # register correct topics for monitoring and control
        self.register()
        
        # initial calibration
        if initial_calib:
            self.calibrate()
        
    def calibrate(self):
        rbt_log_warn(f'Self-calibration in progress...')
        self.go_to_raw_position(255)
        rospy.sleep(1)
        self.calibrated_upper = self.current_status.current_position
        self.go_to_raw_position(0)
        rospy.sleep(1)
        self.calibrated_lower = self.current_status.current_position
        result = [self.calibrated_lower, self.calibrated_upper]
        self.limit = tuple(result)
        rospy.set_param('robotiq_2f_controller/raw_range', result)
        rbt_log_success(f'Calibration completed')
        rbt_log_success(f'Actual gripper range is {self.limit}')

    def register(self, timeout = 10):
        try:
            rospy.wait_for_message(self.status_topic, Robotiq2FInput, timeout=timeout)
            self.status_monitor = rospy.Subscriber(self.status_topic, Robotiq2FInput, callback=self.status_monitor_callback)
            self.command_publisher = rospy.Publisher(self.command_topic, Robotiq2FOutput, queue_size=1)
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
        
    def compensated_publish(self, command, lag = None):
        self.command_publisher.publish(command)
        rospy.sleep(self.publisher_delay if lag is None else lag)
        
    def block(self, enable = False, reporting = False):
        while self.is_moving() and enable:
            if reporting: self.print_status()
            rospy.sleep(0.05)
    
    def deactivate(self, wait = 0.1):
        command = Robotiq2FOutput()
        command.activate = 0
        self.compensated_publish(command)
        while self.current_status.activated != 0:
            rospy.sleep(wait)
        
    def activate(self, wait = 0.1):
        command = Robotiq2FOutput()
        command.activate = 1
        self.compensated_publish(command)
        while self.current_status.gripper_status != 3 or self.current_status.activated != 1:
            rospy.sleep(wait)
        
    def status_monitor_callback(self, msg):
        self.current_status = msg
        
    def print_status(self):
        rbt_log(f'{" STATUS REPORT ".center(45, "=")}')
        rbt_log(f'Jaw opening   : {round(self.conv_p.opening_mm, 2):>6} mm ({round(self.conv_p.opening_in, 2)} in)')
        rbt_log(f'Joint angle   : {round(self.conv_p.rad, 2):>6} rad ({round(self.conv_p.deg, 2)} degrees)')
        rbt_log(f'Gripper speed : {round(self.conv_s.mmps, 2):>6} mm/s ({round(self.conv_s.inps, 2)} in/s)')
        rbt_log(f'Gripper force : {round(self.conv_f.N, 2):>6} N ({round(self.conv_f.lbf, 2)} lbf)')
        rbt_log(f'Motor current : {round(self.current_status.motor_current * 10, 2):>6} mA')
        rbt_log(f'Holding       : {self.is_holding()}')
        rbt_log(f'Moving        : {self.is_moving()}')
        rbt_log(f'{" END REPORT ".center(45, "=")}')
    
    def print_raw_status(self, config = False):
        rbt_log(f'{" RAW STATUS REPORT ".center(45, "=")}')
        rbt_log(f'activated: {self.current_status.activated}')
        rbt_log(f'action_status: {self.current_status.action_status}')
        rbt_log(f'gripper_status: {self.current_status.gripper_status}')
        rbt_log(f'object_status: {self.current_status.object_status}')
        rbt_log(f'fault_status: {self.current_status.fault_status}')
        rbt_log(f'position_request: {self.current_status.position_request}')
        rbt_log(f'current_position: {self.current_status.current_position}')
        rbt_log(f'motor_current: {self.current_status.motor_current}')
        if config:
            rbt_log(f'speed_setting: {self.default_speed}')
            rbt_log(f'force_setting: {self.default_force}')
        rbt_log(f'{" END REPORT ".center(45, "=")}')

    def is_moving(self):
        return (self.current_status.object_status == 0)
    
    def is_holding(self):
        return (self.current_status.object_status == 1 or self.current_status.object_status == 2)
        
    def set_gripper_position(self, position, blocking = True, unit = 'raw', log_status = False):
        # check valid units
        acceptable_units = ['raw', 'mm', 'in', 'rad', 'deg']
        if unit not in acceptable_units:
            rbt_log_error(f'Setting gripper position failed')
            rbt_log_error(f'{unit} is not a valid unit. Expecting {acceptable_units}')
        
        # unit conversion
        if unit == 'deg':
            self.conv_p.deg = position
        elif unit == 'rad':
            self.conv_p.rad = position
        elif unit == 'mm':
            self.conv_p.opening_mm = position
        elif unit == 'in':
            self.conv_p.opening_in = position
        else:
            self.conv_p.raw = position
        
        # actually executing
        self.compensated_publish(self.generate_go_command(self.conv_p.raw))
        self.block(enable=blocking, reporting=log_status)
        
        # wait for status update
        rospy.sleep(0.2)
        
        # final output
        if log_status: self.print_status()
        
    def set_gripper_speed(self, speed, unit = 'raw'):
        acceptable_units = ['raw', 'mm/s', 'in/s']
        if unit not in acceptable_units:
            rbt_log_error(f'Setting gripper speed failed')
            rbt_log_error(f'{unit} is not a valid unit. Expecting {acceptable_units}')
        
        # unit conversion
        if unit == 'mm/s':
            self.conv_s.mmps = speed
        elif unit == 'in/s':
            self.conv_s.inps = speed
        else:
            self.conv_s.raw = speed
            
        # setting execution speed
        self.default_speed = self.conv_s.raw
        
    def set_gripper_force(self, force, unit = 'raw'):
        acceptable_units = ['raw', 'N', 'lbf']
        if unit not in acceptable_units:
            rbt_log_error(f'Setting gripper force failed')
            rbt_log_error(f'{unit} is not a valid unit. Expecting {acceptable_units}')
        
        # unit conversion
        if unit == 'N':
            self.conv_f.N = force
        elif unit == 'lbf':
            self.conv_f.lbf = force
        else:
            self.conv_f.raw = force
            
        # setting execution speed
        self.default_force = self.conv_f.raw
        
    def auto_open(self, blocking = True):
        self.set_gripper_position(self.calibrated_lower, blocking)
        return (self.current_status.current_position == self.calibrated_lower)
    
    def auto_close(self, blocking = True):
        self.set_gripper_position(self.calibrated_upper, blocking)
        return (self.current_status.current_position == self.calibrated_lower)
            
    def generate_go_command(self, position):
        command = Robotiq2FOutput()
        command.activate = 1
        command.goto = 1
        command.position = position
        command.speed = self.default_speed
        command.force = self.default_force
        return command
