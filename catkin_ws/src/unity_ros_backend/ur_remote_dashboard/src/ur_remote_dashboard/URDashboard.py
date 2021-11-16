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
from rosgraph_msgs.msg import Log
from std_msgs.msg import String
from std_srvs.srv import Trigger

# pretty print colors
from ur_remote_dashboard.URMisc import PrintColor, RobotModeMapping, SafetyModeMapping

# rospy stuff
import rospy
from rospy.service import ServiceException, ROSException
    
class URDashboard():
    def __init__(self, name='dashboard', using_gripper=False, using_urscript=False, service_timeout=5):
        self.name  = name
        self.debug = False
        self.service_timeout = service_timeout
        self.subscriber_timeout = self.service_timeout
        self.colors = PrintColor()
        self.robot_mode = -1
        self.safety_mode = -1
        self.last_known_installation = None
        self.last_known_program = None
        self.using_urscript = False
        
        self.register_robot_status()
        
        if using_gripper:
            self.register_robotiq_gripper()
        
        if using_urscript:
            self.register_script_publisher()
            
    # ---------------------------------------------------------------------------- #
    # -------------------- ROBOTIQ GRIPPER (WRIST CONNECTION) -------------------- #
    # ---------------------------------------------------------------------------- #
    
    def register_robotiq_gripper(self):
        raise NotImplementedError
    
    # ---------------------------------------------------------------------------- #
    # ------------------------- DASHBOARD SERVER CONTROL ------------------------- #
    # ---------------------------------------------------------------------------- #
    
    def connect_dashboard(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/connect', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/connect', Trigger)()
            # NOTE: return value is success of trigger, not necessarily mean successful connection
            if response.success:
                self.log_success('Connection to dashboard server successful')
                return response.success
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)')       
        except (ServiceException, ROSException) as error:
            self.log_error('Connection to dashboard unsuccessful')
            self.log_error(error)
            return False
        
    def disconnect_dashboard(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/quit', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/quit', Trigger)()
            if response.success:
                self.log_success('Disconnected from dashboard server')
                return response.success
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)')  
        except (ServiceException, ROSException) as error:
            self.log_error('Disconnection from dashboard server unsuccessful')
            self.log_error(error)
            return False
        
    def spam_reconnect(self, retries=10):
        for i in range(retries):
            self.log(f'    Connecting to dashboard server ({retries - i} attempts left)')
            if self.connect_dashboard():
                return True
            rospy.sleep(rospy.Duration(1))
        self.log_error('Reconnection attempts to dashboard server unsuccessful')
        return False

    def log_to_pendant(self, message):
        request = AddToLogRequest()
        request.message = message
        try:        
            rospy.wait_for_service('/ur_hardware_interface/dashboard/add_to_log', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/add_to_log', AddToLog)(request)
            if response.success:
                self.log_success('Message logged to pendant')
                return response.success
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)')  
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to send message to pendant')
            self.log_error(error)
            return False

    def system_shutdown(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/shutdown', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/shutdown', Trigger)()
            if response.success:
                self.warn('Full system shutdown initated. Goodbye.')
                return response.success
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)')  
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to shutdown system')
            self.log_error(error)
            return False
        
    # ---------------------------------------------------------------------------- #
    # ------------------------------ SET MODE ACTION ----------------------------- #
    # ---------------------------------------------------------------------------- #
    
    def set_robot_mode(self):
        raise NotImplementedError

    # ---------------------------------------------------------------------------- #
    # ---------------------- ROBOT POWER AND MOTION CONTROL ---------------------- #
    # ---------------------------------------------------------------------------- #

    def register_robot_status(self):
        try:
            rospy.wait_for_message('/ur_hardware_interface/robot_mode', RobotMode, timeout=self.service_timeout)
            rospy.wait_for_message('/ur_hardware_interface/safety_mode', SafetyMode, timeout=self.service_timeout)
            self.robot_mode_sub = rospy.Subscriber('/ur_hardware_interface/robot_mode', RobotMode, self.robot_status_callback)
            self.robot_safety_sub = rospy.Subscriber('/ur_hardware_interface/safety', SafetyMode, self.robot_safety_callback)
            self.get_program_state(suppressed=True)
            self.log_success('Registered robot_mode and safety_mode subscriber')
        except ROSException as error:
            self.log_error('Unable to register robot_mode and safety_mode subscriber')            
            self.log_error(error)
            
    def robot_status_callback(self, msg):
        if self.robot_mode != msg.mode:
            self.warn(f'Robot mode is now {RobotModeMapping(msg.mode).name}')
        self.robot_mode = msg.mode
        
    def robot_safety_callback(self, msg):
        if self.safety_mode != msg.mode:
            self.warn(f'Robot mode is now {SafetyModeMapping(msg.mode).name}')
        self.safety_mode = msg.mode

    def release_brake(self, abort=30):
        elapsed = 0 # timer to abort
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/brake_release', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/brake_release', Trigger)()
            if response.success:
                self.warn(f'Waiting for power on and brake release ({abort} seconds)')
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)')  
            while self.robot_mode != 7 and elapsed < abort and not rospy.is_shutdown():
                rospy.sleep(1)
                elapsed += 1
                if elapsed > abort:
                    raise ROSException(f'Wait time exceeded {abort} seconds boot time. Aborting...')
            self.log_success('Robot powered on, brakes released, and ready to program')
            return True
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to fully power on robot')
            self.log_error(error)            
            return False

    def clear_operational_mode(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/clear_operational_mode', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/clear_operational_mode', Trigger)()
            if response.success:
                self.log_success('Operational mode cleared')
                return response.success
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to clear operational mode')
            self.log_error(error)            
            return False

    #TODO: convert robot mode to readable string
    def get_robot_mode(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/get_robot_mode', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode)()
            if response.success:
                self.log(f'Robot  Mode is now {response.robot_mode.mode} ({RobotModeMapping(response.robot_mode.mode).name})')
                return response.robot_mode.mode
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to request robot mode')
            self.log_error(error)
            return -1

    #TODO: convert safety mode to readable string
    def get_safety_mode(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/get_safety_mode', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_safety_mode', GetSafetyMode)()
            if response.success:
                self.log(f'Safety Mode is now {response.safety_mode.mode} ({SafetyModeMapping(response.safety_mode.mode).name})')
                return response.safety_mode.mode
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to request safety mode')
            self.log_error(error)
            return -1
        
    def power_off_arm(self, abort=30):
        elapsed = 0 # timer to abort 
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/power_off', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/power_off', Trigger)()
            if response.success:
                self.warn(f'Waiting for robot system to power off ({abort} seconds)')
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)')  
            while self.robot_mode != 3 and elapsed < abort and not rospy.is_shutdown():
                rospy.sleep(1)
                elapsed += 1
                if elapsed > abort:
                    raise ROSException(f'Wait time exceeded {abort} seconds power off time. Aborting...')
            self.log_success('Robot powered off, brakes engaged')
            return True
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to power off robot')
            self.log_error(error)            
            return False

    def power_on_arm(self, abort=30):
        elapsed = 0 # timer to abort 
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/power_on', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/power_on', Trigger)()
            if response.success:
                self.warn(f'Waiting for robot system to power on ({abort} seconds)')
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)')  
            while self.robot_mode != 5 and elapsed < abort and not rospy.is_shutdown():
                rospy.sleep(1)
                elapsed += 1
                if elapsed > abort:
                    raise ROSException(f'Wait time exceeded {abort} seconds power off time. Aborting...')
            self.log_success('Robot powered off, brakes engaged')
            return True
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to power off robot')
            self.log_error(error)            
            return False
        
    def cold_boot(self):
        self.release_brake()
        
    # TODO: Handle permanent disconnection, need reimplementation
    def restart_safety(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/restart_safety', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/restart_safety', Trigger)()
            if response.success:
                self.log_success('safety fault or violation cleared')
                return response.success
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)')  
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to clear safety violation')
            self.log_error(error)            
            return False

    def unlock_protective_stop(self, abort=30):
        elapsed = 0 # timer to abort 
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/unlock_protective_stop', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/unlock_protective_stop', Trigger)()
            if response.success:
                self.warn(f'Waiting for protective stop to clear ({abort} seconds)')
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)')  
            while self.safety_mode != 1 and elapsed < abort and not rospy.is_shutdown():
                rospy.sleep(1)
                elapsed += 1
                if elapsed > abort:
                    raise ROSException(f'Wait time exceeded {abort} seconds unlocking time. Aborting...')
            self.log_success('Protective stop cleared')
            return True
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to unlock robot protective stop')
            self.log_error(error)            
            return False

    def hand_back_control(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/hand_back_control', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/hand_back_control', Trigger)()
            if response.success:
                self.log_success('external_control node terminated')
                return response.success
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to terminating external_control node')
            self.log_error(error)
            return False
        
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
            if response.success:
                self.log_success('Payload information saved')
                return response.success
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to set payload mass and center of mass')
            self.log_error(error)
            return False

    def set_motion_speed(self, speed):
        request = SetSpeedSliderFractionRequest()
        request.speed_slider_fraction = clip(speed, 0.0, 1.0)
        try:
            rospy.wait_for_service('/ur_hardware_interface/set_speed_slider', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/set_speed_slider', SetSpeedSliderFraction)(request)
            if response.success: 
                self.log_success('Execution speed slider set')
                return response.success
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to set execution speed slider')
            self.log_error(error)
            return False

    def zero_ft_sensor(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/zero_ftsensor', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/zero_ftsensor', Trigger)()
            if response.success:
                self.log_success('Robot ftsensor is 0')
                return response.success
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to zero ftsensor')
            self.log_error(error)
            return False

    # ---------------------------------------------------------------------------- #
    # ------------------------------ PROGRAM CONTROL ----------------------------- #
    # ---------------------------------------------------------------------------- #

    def load_installation(self, filename, wait=10, reconnect_retries=10):
        request = LoadRequest()
        request.filename = filename
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/load_installation', timeout=self.service_timeout)
            rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_installation', Load)(request)
        except (ServiceException, ROSException) as _:
            self.warn('Known dashboard server disconnection occured')
            self.warn(f'Waiting for {wait} seconds for installation to load correctly')
            rospy.sleep(rospy.Duration(wait))
            self.warn(f'Attempting to reconnect to dashboard server ({reconnect_retries} attempts)')
            self.spam_reconnect(reconnect_retries)
            self.close_popup()
        finally:
            self.last_known_installation = filename
            rospy.sleep(1)
            return True

    def load_program(self, filename, wait=10, reconnect_retries=10):
        request = LoadRequest()
        request.filename = filename
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/load_program', timeout=self.service_timeout)
            rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)(request)
        except (ServiceException, ROSException) as _:
            self.warn('Known dashboard server disconnection occured')
            self.warn(f'Waiting for {wait} seconds for program to load correctly')
            rospy.sleep(rospy.Duration(wait))
            self.warn(f'Attempting to reconnect to dashboard server ({reconnect_retries} attempts)')
            self.spam_reconnect(reconnect_retries)
            self.close_popup()
        finally:
            self.last_known_program = filename
            rospy.sleep(1)
            return True
            
    def start_program(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/play', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)()
            if response.success: 
                self.log_success(f'Program {self.last_known_program} is now running')
                rospy.sleep(1)
                return response.success
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to start program')
            self.log_error(error)
            return False
        
    def stop_program(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/stop', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger)()
            if response.success: 
                self.log_success(f'Program {self.last_known_program} is now stopped')
                rospy.sleep(1)
                return response.success
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to stop program')
            self.log_error(error)
            return False

    def pause_program(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/pause', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/pause', Trigger)()
            if response.success: 
                self.log_success(f'Program {self.last_known_program} is now paused')
                rospy.sleep(1)
                return response.success
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to stop program')
            self.log_error(error)
            return False
        
    def is_program_running(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/program_running', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_running', IsProgramRunning)()
            if response.success: 
                self.log(f'Program {self.last_known_program} is {"" if response.program_running else "not"} running')
                return response.program_running
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to request program running state')
            self.log_error(error)
            return False

    def is_program_saved(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/program_saved', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_saved', IsProgramRunning)()
            if response.success: 
                self.log(f'Program {self.last_known_program} is {"" if response.program_saved else "not"} saved')
                return response.program_saved
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to request program save state')
            self.log_error(error)
            return False

    def get_program_state(self, suppressed=False):
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/program_state', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState)()
            if response.success: 
                self.last_known_program = response.program_name
                if not suppressed: self.log_success(f'Program {self.last_known_program} is {response.state.state}')
                return (response.program_name, response.state.state)
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to request program state')
            self.log_error(error)
            return (None, None)
        
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
            if response.success: 
                self.log_success(f'{"Safety popup" if safety else "Popup"} closed')
                return response.success
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            self.log_error(f'Unable to close {"safety" if safety else ""} popup')
            self.log_error(error)
            return False

    def send_popup(self, message):
        request = PopupRequest()
        request.message = message
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/popup', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/popup', Popup)(request)
            if response.success: 
                self.log_success('Popup sent to pendant')
                return response.success
            else:
                raise ROSException('ROS Trigger was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            self.log_error('Unable to send popup')
            self.log_error(error)
            return False
    
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
        if not self.using_urscript:
            self.register_script_publisher()
        pin = int(clip(pin, 0.0, 1.0))
        value = interp(clip(value, 0.0, 10.0), [0.0, 10.0], [0.0, 1.0])
        self.send_urscript_command(f'set_analog_output_domain({pin}, 1)')
        self.send_urscript_command(f'set_analog_out({pin}, {value})')
    
    def set_analog_current(self, pin, value):
        if not self.using_urscript:
            self.register_script_publisher()
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
        self.using_urscript = True
        self.urscript_publisher = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=1)
        timeout_count = 0
        while self.urscript_publisher.get_num_connections() < 1 and not rospy.is_shutdown():
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
    
    def log_error(self, msg):
        print(f'{self.colors.flags["ERROR"]}[REMOTE DASHBOARD] {msg}{self.colors.effects["RESET"]}')
        # rospy.logerr(f'{self.colors.flags["ERROR"]}[REMOTE DASHBOARD] {msg}{self.colors.effects["RESET"]}')
        # rospy.logerr(f'[REMOTE DASHBOARD] {msg}')
    
    def log_success(self, msg):
        print(f'{self.colors.flags["SUCCESS"]}[REMOTE DASHBOARD] {msg}{self.colors.effects["RESET"]}')
        # rospy.loginfo(f'{self.colors.flags["SUCCESS"]}[REMOTE DASHBOARD] {msg}{self.colors.effects["RESET"]}')
 
    def log(self, msg):
        print(f'{self.colors.flags["LOG"]}[REMOTE DASHBOARD] {msg}{self.colors.effects["RESET"]}')
        # rospy.loginfo(f'[REMOTE DASHBOARD] {msg}')
    
    def warn(self, msg):
        print(f'{self.colors.flags["WARNING"]}[REMOTE DASHBOARD] {msg}{self.colors.effects["RESET"]}')
        # rospy.logwarn(f'[REMOTE DASHBOARD] {msg}')
    
    
    