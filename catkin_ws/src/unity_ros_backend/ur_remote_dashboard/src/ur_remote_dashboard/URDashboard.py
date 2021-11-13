#!/usr/bin/env python3

from enum import Enum
import re
from rospy.impl.tcpros_service import Service

# numpy range clipping and interpolation
from numpy import clip, interp

# ur stuff
from ur_msgs.srv import *
from ur_dashboard_msgs.msg import *
from ur_dashboard_msgs.srv import *

# some standard ros stuff
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from std_srvs.srv import Trigger

# pretty print colors
from ur_remote_dashboard.URMisc import PrintColor as pcolor

# rospy stuff
import rospy
from rospy.service import ServiceException, ROSException
    
class URDashboard():
    def __init__(self, name='dashboard', using_gripper=False, using_urscript=False, service_timeout=5):
        self.name  = name
        self.debug = False
        self.service_timeout = service_timeout
        
        if using_gripper:
            self.register_gripper()
        
        if using_urscript:
            self.register_script_publisher()

    # ---------------------------------------------------------------------------- #
    # -------------------- ROBOTIQ GRIPPER (WRIST CONNECTION) -------------------- #
    # ---------------------------------------------------------------------------- #
    
    def register_gripper(self):
        raise NotImplementedError
    
    # ---------------------------------------------------------------------------- #
    # ------------------------- DASHBOARD SERVER CONTROL ------------------------- #
    # ---------------------------------------------------------------------------- #
    
    def connect_dashboard(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/connect', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/connect', Trigger)()
            return response.success
        except (ServiceException, ROSException) as error:
            pass
        
    def disconnect_dashboard(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/quit', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/quit', Trigger)()
        except (ServiceException, ROSException) as error:
            pass
        
    def spam_reconnect(self, retries=10):
        for i in range(retries): 
            if self.connect_dashboard():
                return True
            rospy.sleep(rospy.Duration(1))
        return False

    def log_to_pendant(self, message):
        request = AddToLogRequest()
        request.message = message
        try:        
            rospy.wait_for_service('/ur_hardware_interface/dashboard/add_to_log', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/add_to_log', AddToLog)(request)
        except (ServiceException, ROSException) as error:
            pass

    def system_shutdown(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/shutdown', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/shutdown', Trigger)()
        except (ServiceException, ROSException) as error:
            pass

    # ---------------------------------------------------------------------------- #
    # ---------------------- ROBOT POWER AND MOTION CONTROL ---------------------- #
    # ---------------------------------------------------------------------------- #

    def release_brake(self, wait=20):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/brake_release', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/brake_release', Trigger)()
            if response.success:
                rospy.sleep(wait)
        except (ServiceException, ROSException) as error:
            pass

    def clear_operational_mode(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/clear_operational_mode', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/clear_operational_mode', Trigger)()
        except (ServiceException, ROSException) as error:
            pass

    #TODO: convert robot mode to readable string
    def get_robot_mode(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/get_robot_mode', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode)()
            return response.robot_mode.mode
        except (ServiceException, ROSException) as error:
            pass

    #TODO: convert safety mode to readable string
    def get_safety_mode(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/get_safety_mode', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_safety_mode', GetSafetyMode)()
            return response.safety_mode.mode
        except (ServiceException, ROSException) as error:
            pass
        
    def power_off_arm(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/power_off', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/power_off', Trigger)()
        except (ServiceException, ROSException) as error:
            pass

    def power_on_arm(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/power_on', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/power_on', Trigger)()
        except (ServiceException, ROSException) as error:
            pass
        
    def cold_boot(self):
        self.release_brake()
        
    def restart_safety(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/restart_safety', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/restart_safety', Trigger)()
        except (ServiceException, ROSException) as error:
            pass

    def unlock_protective_stop(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/unlock_protective_stop', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/unlock_protective_stop', Trigger)()
        except (ServiceException, ROSException) as error:
            pass

    def hand_back_control(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/hand_back_control', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/hand_back_control', Trigger)()
        except (ServiceException, ROSException) as error:
            pass
        
    def set_payload(self, mass, cx, cy, cz):
        request = SetPayloadRequest()
        request.center_of_gravity = Vector3()
        request.center_of_gravity.x = cx
        request.center_of_gravity.y = cy
        request.center_of_gravity.z = cz
        request.mass = mass
        try:
            rospy.wait_for_service('/ur_hardware_interface/set_payload', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/set_payload', SetPayload)(request)
        except (ServiceException, ROSException) as error:
            pass

    def set_motion_speed(self, speed):
        request = SetSpeedSliderFractionRequest()
        request.speed_slider_fraction = clip(speed, 0.0, 1.0)
        try:
            rospy.wait_for_service('/ur_hardware_interface/set_speed_slider', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/set_speed_slider', SetSpeedSliderFraction)(request)
        except (ServiceException, ROSException) as error:
            pass

    def zero_ft_sensor(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/zero_ftsensor', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/zero_ftsensor', Trigger)()
        except (ServiceException, ROSException) as error:
            pass

    # ---------------------------------------------------------------------------- #
    # ------------------------------ PROGRAM CONTROL ----------------------------- #
    # ---------------------------------------------------------------------------- #

    def load_installation(self, filename, wait=5, reconnect_retries=10):
        request = LoadRequest()
        request.filename = filename
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/load_installation', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_installation', Load)(request)
        except (ServiceException, ROSException) as error:
            rospy.sleep(rospy.Duration(wait))
            self.spam_reconnect()
            self.close_popup()
        rospy.sleep(1)

    def load_program(self, filename, wait=5, reconnect_retries=10):
        request = LoadRequest()
        request.filename = filename
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/load_program', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)(request)
        except (ServiceException, ROSException) as error:
            rospy.sleep(rospy.Duration(wait))
            self.spam_reconnect(reconnect_retries)
            self.close_popup()
        rospy.sleep(1)

    def start_program(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/play', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)()
        except (ServiceException, ROSException) as error:
            pass
        
    def stop_program(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/stop', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger)()
        except (ServiceException, ROSException) as error:
            pass

    def pause_program(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/pause', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/pause', Trigger)()
        except (ServiceException, ROSException) as error:
            pass
        
    def is_program_running(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/program_running', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_running', IsProgramRunning)()
        except (ServiceException, ROSException) as error:
            pass

    def is_program_saved(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/program_saved', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_saved', IsProgramSaved)()
            return (response.program_name, response.program_saved)
        except (ServiceException, ROSException) as error:
            pass

    def get_program_state(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/program_state', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', ProgramState)()
            return (response.program_name, response.state.state)
        except (ServiceException, ROSException) as error:
            pass
        
    # only available in headless mode, not using
    def resend_robot_program(self):
        raise NotImplementedError
        
    # ---------------------------------------------------------------------------- #
    # ------------------------------- POPUP CONTROL ------------------------------ #
    # ---------------------------------------------------------------------------- #
    
    def close_popup(self, safety=False):
        try:
            if safety:
                rospy.wait_for_service('/ur_hardware_interface/dashboard/close_safety_popup', timeout=self.service_timeout)
                response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/close_safety_popup', Trigger)()
            else:
                rospy.wait_for_service('/ur_hardware_interface/dashboard/close_popup', timeout=self.service_timeout)
                response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/close_popup', Trigger)()
        except (ServiceException, ROSException) as error:
            pass

    def send_popup(self, message):
        request = PopupRequest()
        request.message = message
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/popup', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/popup', Popup)(request)
        except (ServiceException, ROSException) as error:
            pass
    
    # ---------------------------------------------------------------------------- #
    # -------------------------------- I/O CONTROL ------------------------------- #
    # ---------------------------------------------------------------------------- #

    #TODO: add strict control (if/else) of input to match configurations
    def set_io(self, function, pin, state):
        request = SetIORequest()
        request.fun = function
        request.pin = pin
        request.state = state
        try:
            rospy.wait_for_service('/ur_hardware_interface/set_io', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)(request)
        except (ServiceException, ROSException) as error:
            pass
        
    # workaround for https://github.com/ros-industrial/ur_msgs/issues/11
    def set_analog_voltage(self, pin, value):
        pin = int(clip(pin, 0.0, 1.0))
        value = interp(clip(value, 0.0, 10.0), [0.0, 10.0], [0.0, 1.0])
        self.send_urscript_command(f'set_analog_output_domain({pin}, 1)')
        self.send_urscript_command(f'set_analog_out({pin}, {value})')
    
    def set_analog_current(self, pin, value):
        pin = int(clip(pin, 0.0, 1.0))
        value = interp(clip(value, 4.0, 20.0), [4.0, 20.0], [0.0, 1.0])
        self.send_urscript_command(f'set_analog_output_domain({pin}, 0)')
        self.send_urscript_command(f'set_analog_out({pin}, {value})')
        
    # ---------------------------------------------------------------------------- #
    # ---------------------------- SCRIPTING INTERFACE --------------------------- #
    # ---------------------------------------------------------------------------- #
        
    def send_urscript_command(self, command):
        request = String()
        request.data = 'sec raw_ros_request():\n'
        request.data += '    ' + command + '\n'
        request.data += 'end'
        self.urscript_publisher.publish(request)
        rospy.sleep(0.5)
        
    def send_urscript(self, urp):
        raise NotImplementedError

    def register_script_publisher(self, timeout=10):
        self.urscript_publisher = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=1)
        timeout_count = 0
        while self.urscript_publisher.get_num_connections() < 1:
            timeout_count += 1
            if timeout_count > timeout:
                break
            rospy.sleep(1)
            
    # def send_raw_request(self, query):
    #     request = RawRequestRequest()
    #     request.query = query
    #     try:
    #         rospy.wait_for_service('/ur_hardware_interface/dashboard/raw_request', timeout=self.service_timeout)
    #         response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/raw_request', RawRequest)(request)
    #     except (ServiceException, ROSException) as error:
    #         pass

    # ---------------------------------------------------------------------------- #
    # ------------------------------ READABLE OUTPUT ----------------------------- #
    # ---------------------------------------------------------------------------- #