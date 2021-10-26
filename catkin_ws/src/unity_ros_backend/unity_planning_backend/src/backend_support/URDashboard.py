#!/usr/bin/env python3

from pprint import pprint
from enum import Enum
from rospy.service import ServiceException

from ur_msgs.srv import *

from ur_dashboard_msgs.msg import *
from ur_dashboard_msgs.srv import *

from geometry_msgs.msg import Vector3
from std_msgs.msg import String

from std_srvs.srv import Trigger

import rospy

class DirectEnum(Enum):
    def __repr__(self):
        return self.value
    
    def __str__(self):
        return str(self.value)

class RobotModeMapping(Enum):
    NO_CONTROLLER=-1
    DISCONNECTED=0
    CONFIRM_SAFETY=1
    BOOTING=2
    POWER_OFF=3
    POWER_ON=4
    IDLE=5
    BACKDRIVE=6
    RUNNING=7
    UPDATING_FIRMWARE=8
    
    def __str__(self):
        return f'Robot mode is {self.value} = {self.name}'
    
class SafetyModeMapping(Enum):
    NORMAL=1
    REDUCED=2
    PROTECTIVE_STOP=3
    RECOVERY=4
    SAFEGUARD_STOP=5
    SYSTEM_EMERGENCY_STOP=6
    ROBOT_EMERGENCY_STOP=7
    VIOLATION=8
    FAULT=9
    VALIDATE_JOINT_ID=10
    UNDEFINED_SAFETY_MODE=11
    AUTOMATIC_MODE_SAFEGUARD_STOP=12
    SYSTEM_THREE_POSITION_ENABLING_STOP=13
    
    def __str__(self):
        return f'Safety mode is {self.value} = {self.name}'
    
class IOFunctions():
    SET_DIGITAL_OUT = 1
    SET_FLAG = 2
    SET_ANALOG_OUT = 3
    SET_TOOL_VOLTAGE = 4
    
class DigitalIOMapping():
    pass

class AnalogIOMapping():
    pass
    
class IOStates():
    OFF = 0
    ON = 1
    
class ToolStates():
    VOLTAGE_0V = 0
    VOLTAGE_12V = 12
    VOLTAGE_24V = 24
    
class URDashboard():
    def __init__(self, name='TeachPendant'):
        self.name  = name
        self.debug = False
        self.command_publisher = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=1)
        
        while self.command_publisher.get_num_connections() < 1:
            print('Waiting for a connection...')
            rospy.sleep(rospy.Duration(1))
    
    def set_debug(self, debugging=False):
        self.debug = debugging
        
    # TODO: Add delay for booting time
    def power_on_arm(self):
        """Power on the robot motors. To fully start the robot, call release_brakes() or trigger brake_release service afterwards."""
        rospy.wait_for_service('/ur_hardware_interface/dashboard/power_on')
        rospy.ServiceProxy('/ur_hardware_interface/dashboard/power_on', Trigger)()
        
    def power_off_arm(self):
        """Power off the robot motors."""
        rospy.wait_for_service('/ur_hardware_interface/dashboard/power_off')
        rospy.ServiceProxy('/ur_hardware_interface/dashboard/power_off', Trigger)()
        
    def release_brakes(self):
        """Service to release the brakes. If the robot is currently powered off, it will get powered on on the fly."""
        rospy.wait_for_service('/ur_hardware_interface/dashboard/brake_release')
        rospy.ServiceProxy('/ur_hardware_interface/dashboard/brake_release', Trigger)()
            
    def log_pendant(self, msg):
        """Service to add a message to the robot's log. View in Log on Teach Pendant."""
        rospy.wait_for_service('/ur_hardware_interface/dashboard/add_to_log')
        rospy.ServiceProxy('/ur_hardware_interface/dashboard/add_to_log', AddToLog)(msg)
        
    def clear_operational_mode(self):
        """If this service is called the operational mode can again be changed from PolyScope, and the user password is enabled."""   
        rospy.wait_for_service('/ur_hardware_interface/dashboard/clear_operational_mode')
        rospy.ServiceProxy('/ur_hardware_interface/dashboard/clear_operational_mode', Trigger)()

    def close_popup(self, safety=False):
        """Close a (non-safety) popup on the teach pendant."""
        if safety:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/close_safety_popup')
            rospy.ServiceProxy('/ur_hardware_interface/dashboard/close_safety_popup', Trigger)()
        else:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/close_popup')
            rospy.ServiceProxy('/ur_hardware_interface/dashboard/close_popup', Trigger)()
            
    def connect(self):
        """Service to reconnect to the dashboard server."""
        rospy.wait_for_service('/ur_hardware_interface/dashboard/connect')
        resp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/connect', Trigger)()
        return resp.success
        
    def disconnect(self):
        """Disconnect from the dashboard service."""
        rospy.wait_for_service('/ur_hardware_interface/dashboard/quit')
        rospy.ServiceProxy('/ur_hardware_interface/dashboard/quit', Trigger)()
            
    def get_loaded_program(self):
        """Get full path of loaded program."""
        rospy.wait_for_service('/ur_hardware_interface/dashboard/get_loaded_program')
        resp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_loaded_program', GetLoadedProgram)()
        return resp.program_name
    
    def get_robot_mode(self, report=False):
        """Service to query the current safety mode

        Args:
            report (bool, optional): Print to terminal if True. Defaults to False.

        Returns:
            int: numerical value of robot mode, refer to RobotModeMapping enum.
        
        Modes:
            1: NO_CONTROLLER
            2: DISCONNECTED
            3: CONFIRM_SAFETY
            4: BOOTING
            5: POWER_OFF
            6: POWER_ON
            7: IDLE
            8: BACKDRIVE
            9: RUNNING
            10: UPDATING_FIRMWARE
        """
        rospy.wait_for_service('/ur_hardware_interface/dashboard/get_robot_mode')
        resp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode)()
        if report: print(RobotModeMapping(resp.robot_mode.mode))
        return resp.robot_mode.mode

    def get_safety_mode(self, report=False):
        """Service to query the current safety mode

        Args:
            report (bool, optional): Print to terminal if True. Defaults to False.

        Returns:
            int: numerical value of safety mode, refer to SafetyModeMapping enum.
            
        Modes:
            1:NORMAL
            2:REDUCED
            3:PROTECTIVE_STOP
            4:RECOVERY
            5:SAFEGUARD_STOP
            6:SYSTEM_EMERGENCY_STOP
            7:ROBOT_EMERGENCY_STOP
            8:VIOLATION
            9:FAULT
            10:VALIDATE_JOINT_ID
            11:UNDEFINED_SAFETY_MODE
            12:AUTOMATIC_MODE_SAFEGUARD_STOP
            13:SYSTEM_THREE_POSITION_ENABLING_STOP
        """
        rospy.wait_for_service('/ur_hardware_interface/dashboard/get_safety_mode')
        resp =  rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_safety_mode', GetSafetyMode)()
        if report: print(SafetyModeMapping(resp.safety_mode.mode))
        return resp.safety_mode.mode
    
    def load_installation(self, filename, wait=10, reconnect_retries=10):
        """Load a robot installation from a file."""
        req = LoadRequest()
        req.filename = filename
        rospy.wait_for_service('/ur_hardware_interface/dashboard/load_installation')
        try:
            rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_installation', Load)(req)
        except ServiceException as e:
            print('Known disconnection error occured. Ignoring.')
            print(f'Waiting for {wait} seconds so installation load correctly.')
            for _ in range(wait):
                rospy.sleep(rospy.Duration(1))
                self.close_popup()
            self.spam_reconnect(reconnect_retries)

    def load_program(self, filename):
        """Load a robot program from a file."""
        req = LoadRequest()
        req.filename = filename
        rospy.wait_for_service('/ur_hardware_interface/dashboard/load_program')
        rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)(req)

    def pause_program(self):
        """Pause a running program."""
        rospy.wait_for_service('/ur_hardware_interface/dashboard/pause')
        self.Pause_serv = rospy.ServiceProxy('/ur_hardware_interface/dashboard/pause', Trigger)()

    def start_program(self):
        """Start execution of a previously loaded program."""
        rospy.wait_for_service('/ur_hardware_interface/dashboard/play')
        rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)()

    def send_popup(self, msg):
        """Service to show a popup with content = msg on the UR Teach pendant."""
        rospy.wait_for_service('/ur_hardware_interface/dashboard/popup')
        rospy.ServiceProxy('/ur_hardware_interface/dashboard/popup', Popup)(msg)

    def check_program_running(self, report=False, request_name=False):
        """Query whether there is currently a program running."""
        rospy.wait_for_service('/ur_hardware_interface/dashboard/program_running')
        resp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_running', IsProgramRunning)()
        if report: 
            print(f'Currently {"" if resp.program_running else "not"} running a program.')
            if request_name:
                print(f'Program running is {self.get_loaded_program()}')
        return resp.program_running

    def is_program_saved(self):
        """Query whether the current program is saved."""
        rospy.wait_for_service('/ur_hardware_interface/dashboard/program_saved')
        resp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_saved', IsProgramSaved)
        return resp.program_saved

    def get_program_state(self):
        """Service to query the current program state.
        
        States:
            STOPPED: not running
            PAUSED: running but not executing
            PLAYING: executing
        """
        rospy.wait_for_service('/ur_hardware_interface/dashboard/program_state')
        resp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState)()
        return resp.state.state

    def raw_request(self, req):
        """General purpose service to send arbitrary messages (not explicitly supported) to the dashboard server.

        Args:
            req (str): a query for the dashboard
        """
        rospy.wait_for_service('/ur_hardware_interface/dashboard/raw_request')
        rospy.ServiceProxy('/ur_hardware_interface/dashboard/raw_request', RawRequest)(req)

    def reset_safety(self):
        """Used when robot gets a safety fault or violation to restart the safety. 
        
        After safety has been rebooted the robot will be in Power Off. 
        
        Warning: You should always ensure it is okay to restart the system. 
        
        It is highly recommended to check the error log before using this command (either via PolyScope or e.g. ssh connection).
        """
        rospy.wait_for_service('/ur_hardware_interface/dashboard/restart_safety')
        rospy.ServiceProxy('/ur_hardware_interface/dashboard/restart_safety', Trigger)()

    def shutdown(self):
        """Shutdown the robot controller."""
        rospy.wait_for_service('/ur_hardware_interface/dashboard/shutdown')
        rospy.ServiceProxy('/ur_hardware_interface/dashboard/shutdown', Trigger)()

    def stop_program(self):
        """Stop program execution on the robot."""
        rospy.wait_for_service('/ur_hardware_interface/dashboard/stop')
        rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger)()

    def unlock_protective_stop(self):
        """Dismiss a protective stop to continue robot movements. 
        
        Warning: It is the responsibility of the user to ensure the cause of the protective stop is resolved before calling this service.
        """
        rospy.wait_for_service('/ur_hardware_interface/dashboard/unlock_protective_stop')
        rospy.ServiceProxy('/ur_hardware_interface/dashboard/unlock_protective_stop', Trigger)()

    def hand_back_control(self):
        """Calling this service will make the "External Control" program node on the UR-Program return."""
        rospy.wait_for_service('/ur_hardware_interface/hand_back_control')
        rospy.ServiceProxy('/ur_hardware_interface/hand_back_control', Trigger)()

    def set_io(self, function, pin, state, using_voltage=True):
        """Service to set any of the robot's IOs

        Args:
            function (int): the function the service should perform
            pin (int): the IO pin to perform function on
            state (float): analog (range) or digital state of pin/flags
            using_voltage (boolean): only applies when setting analog state. Default True (using voltage instead of current, unit Amps).
        
        Limits:
             analog output current: 4-20 mA
             analog output voltage: 0-10 V
        
        Functions:
            1: FUN_SET_DIGITAL_OUT
            2: FUN_SET_FLAG
            3: FUN_SET_ANALOG_OUT
            4: FUN_SET_TOOL_VOLTAGE
        
        Pins: Refer to manual or https://www.universal-robots.com/articles/ur/interface-communication/connecting-internal-inputs-and-outputs-io-on-the-robots-controller/
        
        State:
            setting digital IO or flags:
                0: STATE_OFF
                1: STATE_ON

            setting tool voltage:
                0: STATE_TOOL_VOLTAGE_0V
                12: STATE_TOOL_VOLTAGE_12V
                24: STATE_TOOL_VOLTAGE_24V
        """
        req = SetIORequest()
        req.fun = function
        
        # handle voltage or current for analog pins
        if function == IOFunctions.SET_ANALOG_OUT:
            if using_voltage:
                self.send_raw_ur_command(f'set_analog_outputdomain({pin},1)')
            else:
                self.send_raw_ur_command(f'set_analog_outputdomain({pin},0)')
        
        req.pin = pin
        req.state = float(state)
        rospy.wait_for_service('/ur_hardware_interface/set_io')
        rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)(req)

    def set_payload(self, mass, cx, cy, cz):
        """Setup the mounted payload through a ROS service.

        Args:
            mass (float): mass of payload in kg
            cx (float): x value of center of gravity
            cy (float): y value of center of gravity
            cz (float): z value of center of gravity
        """
        req = SetPayloadRequest()
        cog = Vector3()
        cog.x = cx
        cog.y = cy
        cog.z = cz
        req.mass = mass
        req.center_of_gravity = cog
        rospy.wait_for_service('/ur_hardware_interface/set_payload')
        rospy.ServiceProxy('/ur_hardware_interface/set_payload', SetPayload)(req)

    def zero_ft_sensor(self):
        """Calling this service will zero the robot's ftsensor. 
        
        Note: On e-Series robots this will only work when the robot is in remote-control mode.
        """
        rospy.wait_for_service('/ur_hardware_interface/set_speed_slider')
        self.SetSpeedSlider_serv = rospy.ServiceProxy('/ur_hardware_interface/set_speed_slider', SetSpeedSliderFraction)
        
    def send_raw_ur_command(self, command):
        self.command_publisher.publish(String(command))
        
    def send_raw_urscript(self, script):
        raise NotImplemented
    
    def cold_boot(self, wait=10):
        """Go directly to operational mode from power up."""
        self.release_brakes()
        print('Waiting for {wait} seconds so robot started correctly.')
        rospy.sleep(rospy.Duration(wait))
        
    def spam_reconnect(self, retries=10):
        for _ in range(retries): 
            print("Attempting to reconnect to dashboard server...")
            success = self.connect()
            if success:
                break
            rospy.sleep(rospy.Duration(1))
            
    def enable_freedrive():
        """Enable manual control of the robot equivalent to pressing Free Drive button on Teach Pendant."""
        raise NotImplementedError
    
    def disable_freedrive():
        """Disable manual manuevring of the arm (Free Drive)"""
        raise NotImplementedError