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
from ur_remote_dashboard.URMisc import *

# robotiq commander
from robotiq_ros_controller.RobotiqCommander import Robotiq2FCommander

# rospy stuff
import rospy
from rospy.service import ServiceException, ROSException
    
class URDashboard():
    """Class to handle communication, control, and status monitoring of the UR Remote Control dashboard server."""
    
    def __init__(self, name:str = 'dashboard', using_gripper:bool = False, using_urscript:bool = False, service_timeout:int = 5) -> None:
        """Class constructor.

        Args:
            name (str, optional): Readable name of the dashboard, not important. Defaults to 'dashboard'.
            using_gripper (bool, optional): When True, the dashboard will try to connect to the gripper. Defaults to False.
            using_urscript (bool, optional): When True, the dashboard will register a publisher to run urscripts. Defaults to False.
            service_timeout (int, optional): Default timeout of waiting for messages or services. Defaults to 5.
        """
        
        # pretty print colors
        self.colors = PrintColor()
        
        # assigning instance variables
        self.name  = name
        self.service_timeout = service_timeout
        self.subscriber_timeout = self.service_timeout
        self.robot_mode = -1
        self.safety_mode = -1
        self.last_known_installation = None
        self.last_known_program = None
        self.using_urscript = False
        
        # register robot first state
        self.register_robot_status()
        
        # register requested component from constructor
        if using_gripper:
            self.register_robotiq_gripper()
        
        if using_urscript:
            self.register_script_publisher()
            
    # ---------------------------------------------------------------------------- #
    # -------------------- ROBOTIQ GRIPPER (WRIST CONNECTION) -------------------- #
    # ---------------------------------------------------------------------------- #
    
    def register_robotiq_gripper(self):
        self.gripper = Robotiq2FCommander()
    
    # ---------------------------------------------------------------------------- #
    # ------------------------- DASHBOARD SERVER CONTROL ------------------------- #
    # ---------------------------------------------------------------------------- #
    
    def connect_dashboard(self) -> bool:
        """[Re]connect to the dashboard server.

        Raises:
            ROSException: raised when ROS service call return success = False.
            ServiceException: raised when service call failed.

        Returns:
            bool: True if connection is re-established, False otherwise.
        
        Note: 
            Return value is success of trigger, not necessarily mean successful connection.
            Success usually mean the action is completed, assuming proper ROS connection.
        """
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/connect', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/connect', Trigger)()
            if response.success:
                ur_log_success('Connection to dashboard server successful')
                return response.success
            else:
                raise ROSException('Service call was not successful (response.success = False)')       
        except (ServiceException, ROSException) as error:
            ur_log_error('Connection to dashboard unsuccessful')
            ur_log_error(error)
            return False
        
    def disconnect_dashboard(self) -> bool:
        """Disconnect from the dashboard server.

        Raises:
            ROSException: raised when ROS service call return success = False.
            ServiceException: raised when service call failed.

        Returns:
            bool: True if connection is closed, False otherwise.
            
        Note: 
            Return value is success of trigger, not necessarily mean successful connection.
            Success usually mean the action is completed, assuming proper ROS connection.
        """
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/quit', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/quit', Trigger)()
            if response.success:
                ur_log_success('Disconnected from dashboard server')
                return response.success
            else:
                raise ROSException('Service call was not successful (response.success = False)')  
        except (ServiceException, ROSException) as error:
            ur_log_error('Disconnection from dashboard server unsuccessful')
            ur_log_error(error)
            return False
        
    def spam_reconnect(self, retries:int = 10) -> bool:
        """Multiple retries calling connect_dashboard() to re-establish dashboard server connection.

        Args:
            retries (int, optional): Number of retries. Defaults to 10.

        Returns:
            bool: True if reconnection is successful. False otherwise.
            
        Note: 
            Return value is success of trigger, not necessarily mean successful connection.
            Success usually mean the action is completed, assuming proper ROS connection.
        """
        for i in range(retries):
            ur_log(f'    Connecting to dashboard server ({retries - i} attempts left)')
            if self.connect_dashboard():
                return True
            rospy.sleep(rospy.Duration(1))
        ur_log_error('Reconnection attempts to dashboard server unsuccessful')
        return False

    def log_to_pendant(self, message:str) -> bool:
        """Send a message to the TeachPendant log

        Args:
            message (str): the message.

        Raises:
            ROSException: raised when ROS service call return success = False.
            ServiceException: raised when service call failed.

        Returns:
            bool: True if message is registered to the pendant log.
            
        Note: 
            Return value is success of trigger, not necessarily mean successful message logging.
            Success usually mean the action is completed, assuming proper ROS connection.
        """
        request = AddToLogRequest()
        request.message = message
        try:        
            rospy.wait_for_service('/ur_hardware_interface/dashboard/add_to_log', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/add_to_log', AddToLog)(request)
            if response.success:
                ur_log_success('Message logged to pendant')
                return response.success
            else:
                raise ROSException('Service call not successful (response.success = False)')  
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to send message to pendant')
            ur_log_error(error)
            return False

    def system_shutdown(self, close_rospy:bool = False) -> bool:
        """Shutdown the enture system completely (arm, control box, dashboard server).
        
        Args:
            close_rospy (bool, optional): shutdown rospy along with the system. Dafault to False.

        Raises:
            ROSException: raised when ROS service call return success = False.
            ServiceException: raised when service call failed.

        Returns:
            bool: True if the system shutdown is successful. False otherwise.
            
        Note: 
            Return value is success of trigger, not necessarily mean successful connection.
            Success usually mean the action is completed, assuming proper ROS connection.
        """
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/shutdown', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/shutdown', Trigger)()
            if response.success:
                ur_log_warn('Full system shutdown initated. Goodbye.')
                ur_log_warn(f'Waiting for system to completely shutdown ({5} seconds).')
                if close_rospy: rospy.signal_shutdown('System-wide shutdown initiated.')
                rospy.sleep(5)
                return response.success
            else:
                raise ROSException('Service call was not successful (response.success = False)')  
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to shutdown system')
            ur_log_error(error)
            return False
        
    # ---------------------------------------------------------------------------- #
    # ------------------------------ SET MODE ACTION ----------------------------- #
    # ---------------------------------------------------------------------------- #
    
    def set_robot_mode(self):
        raise NotImplementedError

    # ---------------------------------------------------------------------------- #
    # ---------------------- ROBOT POWER AND MOTION CONTROL ---------------------- #
    # ---------------------------------------------------------------------------- #

    def register_robot_status(self) -> None:
        """Register necessary subscribers and callbacks to monitor robot operational status."""
        try:
            # wait for topics to show up
            rospy.wait_for_message('/ur_hardware_interface/robot_mode', RobotMode, timeout=self.service_timeout)
            rospy.wait_for_message('/ur_hardware_interface/safety_mode', SafetyMode, timeout=self.service_timeout)
            
            # register robot and safety state subscriber with appropriate callbacks
            self.robot_mode_sub = rospy.Subscriber('/ur_hardware_interface/robot_mode', RobotMode, self.robot_status_callback)
            self.robot_safety_sub = rospy.Subscriber('/ur_hardware_interface/safety', SafetyMode, self.robot_safety_callback)
            
            # get the name of the loaded program and its state at least once, surpressing the output
            self.get_program_state(suppressed=True)
            ur_log_success('Registered robot_mode and safety_mode subscriber')
        except ROSException as error:
            ur_log_error('Unable to register robot_mode and safety_mode subscriber')            
            ur_log_error(error)
            
    def robot_status_callback(self, msg:RobotMode) -> None:
        """Callback to robot mode subscriber, update last known robot mode.

        Args:
            msg (RobotMode): ROS message containing current robot mode.
        """
        if self.robot_mode != msg.mode:
            ur_log_warn(f'Robot mode is now {RobotModeMapping(msg.mode).name}')
        self.robot_mode = msg.mode
        
    def robot_safety_callback(self, msg:SafetyMode) -> None:
        """Callback to safety mode subscriber, update last known robot safety mode.

        Args:
            msg (SafetyMode): ROS message containing current robot safety mode.
        """
        if self.safety_mode != msg.mode:
            ur_log_warn(f'Robot mode is now {SafetyModeMapping(msg.mode).name}')
        self.safety_mode = msg.mode

    def release_brake(self, abort:int = 30) -> bool:
        """Release the brakes. If the arm is not powered, it will be powered on.
        
        Equivalent to setting robot state to 7.

        Args:
            abort (int, optional): Wait time for powering on and robot state to update. Defaults to 30.

        Raises:
            ROSException: When the wait time exceed abort time for robot to update its status to 7 (RUNNING).
            ROSException: When service call returned success = False.
            ServiceException: When service call failed.

        Returns:
            bool: True if the robot is powered on properly and brakes are released. False otherwise.
            
        Note: 
            Return value is success of trigger, not necessarily mean successful connection.
            Success usually mean the action is completed, assuming proper ROS connection.
        """
        elapsed = 0 # timer to abort
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/brake_release', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/brake_release', Trigger)()
            if response.success:
                ur_log_warn(f'Waiting for power on and brake release ({abort} seconds)')
            else:
                raise ROSException('Service call was not successful (response.success = False)')  
            while self.robot_mode != 7 and elapsed < abort and not rospy.is_shutdown():
                rospy.sleep(1)
                elapsed += 1
                if elapsed > abort:
                    raise ROSException(f'Wait time exceeded {abort} seconds boot time. Aborting...')
            ur_log_success('Robot powered on, brakes released, and ready to program')
            return True
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to fully power on robot')
            ur_log_error(error)            
            return False

    def clear_operational_mode(self) -> bool:
        """If this service is called the operational mode can again be changed from PolyScope, and the user password is enabled.

        Raises:
            ROSException: When service call returned success = False.
            ServiceException: When service call failed.

        Returns:
            bool: True if operational mode is cleared. False otherwise.

        Note: 
            Return value is success of trigger, not necessarily mean successful connection.
            Success usually mean the action is completed, assuming proper ROS connection.
        """
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/clear_operational_mode', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/clear_operational_mode', Trigger)()
            if response.success:
                ur_log_success('Operational mode cleared')
                return response.success
            else:
                raise ROSException('Service call was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to clear operational mode')
            ur_log_error(error)            
            return False

    def get_robot_mode(self) -> int:
        """Request the current robot mode. May be redundant due to RobotState subscriber.
        See URMisc.RobotModeMapping for more details.

        Raises:
            ROSException: When service call returned success = False.
            ServiceException: When service call failed.

        Returns:
            int: Current robot mode.
            
        Note: 
            Return value is success of trigger, not necessarily mean successful connection.
            Success usually mean the action is completed, assuming proper ROS connection.
        """
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/get_robot_mode', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode)()
            if response.success:
                ur_log(f'Robot  Mode is now {response.robot_mode.mode} ({RobotModeMapping(response.robot_mode.mode).name})')
                return response.robot_mode.mode
            else:
                raise ROSException('Service call was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to request robot mode')
            ur_log_error(error)
            return -1

    def get_safety_mode(self) -> None:
        """Request the current robot safety mode. May be redundant due to SafetyMode subscriber.
        See URMisc.SafetyModeMapping for more details.

        Raises:
            ROSException: When service call returned success = False.
            ServiceException: When service call failed.

        Returns:
            int: Current robot safety mode.
        """
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/get_safety_mode', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_safety_mode', GetSafetyMode)()
            if response.success:
                ur_log(f'Safety Mode is now {response.safety_mode.mode} ({SafetyModeMapping(response.safety_mode.mode).name})')
                return response.safety_mode.mode
            else:
                raise ROSException('Service call was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to request safety mode')
            ur_log_error(error)
            return -1
        
    def power_off_arm(self, abort=30):
        """Power off the arm. Equivalent to setting robot state to 3.

        Args:
            abort (int, optional): Wait time for powering off and robot state to update. Defaults to 30.

        Raises:
            ROSException: When the wait time exceed abort time for robot to update its status to 3 (POWER_OFF).
            ROSException: When service call returned success = False.
            ServiceException: When service call failed.

        Returns:
            bool: True if the robot is powered off properly. False otherwise.
            
        Note: 
            Return value is success of trigger, not necessarily mean successful connection.
            Success usually mean the action is completed, assuming proper ROS connection.
        """
        elapsed = 0 # timer to abort 
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/power_off', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/power_off', Trigger)()
            if response.success:
                ur_log_warn(f'Waiting for robot system to power off ({abort} seconds)')
            else:
                raise ROSException('Service call was not successful (response.success = False)')  
            while self.robot_mode != 3 and elapsed < abort and not rospy.is_shutdown():
                rospy.sleep(1)
                elapsed += 1
                if elapsed > abort:
                    raise ROSException(f'Wait time exceeded {abort} seconds power off time. Aborting...')
            ur_log_success('Robot powered off, brakes engaged')
            return True
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to power off robot')
            ur_log_error(error)            
            return False

    def power_on_arm(self, abort=30):
        """Power on the arm. Equivalent to setting robot state to 4.

        Args:
            abort (int, optional): Wait time for powering on and robot state to update. Defaults to 30.

        Raises:
            ROSException: When the wait time exceed abort time for robot to update its status to 4 (POWER_ON).
            ROSException: When service call returned success = False.
            ServiceException: When service call failed.

        Returns:
            bool: True if the robot is powered on properly. False otherwise.
            
        Note: 
            Return value is success of trigger, not necessarily mean successful connection.
            Success usually mean the action is completed, assuming proper ROS connection.
        """
        elapsed = 0 # timer to abort 
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/power_on', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/power_on', Trigger)()
            if response.success:
                ur_log_warn(f'Waiting for robot system to power on ({abort} seconds)')
            else:
                raise ROSException('Service call was not successful (response.success = False)')  
            while self.robot_mode != 5 and elapsed < abort and not rospy.is_shutdown():
                rospy.sleep(1)
                elapsed += 1
                if elapsed > abort:
                    raise ROSException(f'Wait time exceeded {abort} seconds power on time. Aborting...')
            ur_log_success('Robot powered on, brakes engaged')
            return True
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to power on robot')
            ur_log_error(error)            
            return False
        
    def cold_boot(self) -> None:
        """Wrapper function for release_brake(). Name made more sense from cold boot."""
        self.release_brake()
        
    def restart_safety(self) -> bool:
        """Clear robot safety fault or violation to restart the safety. Robot will be in state 3 (POWER_OFF).
        
        TODO: Handle permanent disconnection, need reimplementation
        
        Note:
            You should always ensure it is okay to restart the system. 
            It is highly recommended to check the error log BEFORE 
            using this command (either via PolyScope or e.g. ssh connection).

        Raises:
            ROSException: When service call returned success = False.
            ServiceException: When service call failed.

        Returns:
            bool: True if violations are cleared. False otherwise.
            
        Note: 
            Return value is success of trigger, not necessarily mean successful connection.
            Success usually mean the action is completed, assuming proper ROS connection.
        """
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/restart_safety', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/restart_safety', Trigger)()
            if response.success:
                ur_log_success('safety fault or violation cleared')
                return response.success
            else:
                raise ROSException('Service call was not successful (response.success = False)')  
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to clear safety violation')
            ur_log_error(error)            
            return False

    def unlock_protective_stop(self, abort:int = 30) -> bool:
        """Dismiss a protective stop to continue robot movements.
        
        Equivalent to waiting for safety mode to be 1 (NORMAL).

        Args:
            abort (int, optional): Wait time for protective stop to clear. Defaults to 30.

        Raises:
            ROSException: When the wait time exceed abort time to unlock stop.
            ROSException: When service call returned success = False.
            ServiceException: When service call failed.

        Returns:
            bool: True if protective stop is cleared. False otherwise.
            
        Note: 
            Return value is success of trigger, not necessarily mean successful connection.
            Success usually mean the action is completed, assuming proper ROS connection.
        """
        elapsed = 0 # timer to abort 
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/unlock_protective_stop', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/unlock_protective_stop', Trigger)()
            if response.success:
                ur_log_warn(f'Waiting for protective stop to clear ({abort} seconds)')
            else:
                raise ROSException('Service call was not successful (response.success = False)')  
            while self.safety_mode != 1 and elapsed < abort and not rospy.is_shutdown():
                rospy.sleep(1)
                elapsed += 1
                if elapsed > abort:
                    raise ROSException(f'Wait time exceeded {abort} seconds unlocking time. Aborting...')
            ur_log_success('Protective stop cleared')
            return True
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to unlock robot protective stop')
            ur_log_error(error)            
            return False

    def hand_back_control(self) -> bool:
        """Make the "External Control" program node on the UR-Program return.
        Program may finish and stop at this point.

        Raises:
            ROSException: When service call returned success = False.
            ServiceException: When service call failed.

        Returns:
            bool: True if external_control node is terminated. False otherwise.
            
        Note: 
            Return value is success of trigger, not necessarily mean successful connection.
            Success usually mean the action is completed, assuming proper ROS connection.
        """
        try:
            rospy.wait_for_service('/ur_hardware_interface/hand_back_control', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/hand_back_control', Trigger)()
            if response.success:
                ur_log_success('external_control node terminated')
                return response.success
            else:
                raise ROSException('Service call was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to terminating external_control node')
            ur_log_error(error)
            return False
        
    def set_payload(self, mass:float, cx:float, cy:float, cz:float) -> bool:
        """Set the payload mass and centoer of mass.

        Args:
            mass (float): mass of the payload
            cx (float): center of mass point along the x-axis
            cy (float): center of mass point along the y-axis
            cz (float): center of mass point along the z-axis

        Raises:
            ROSException: When service call returned success = False.
            ServiceException: When service call failed.

        Returns:
            bool: True if payload information is set. False otherwise.
        """
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
                ur_log_success('Payload information saved')
                return response.success
            else:
                raise ROSException('Service call was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to set payload mass and center of mass')
            ur_log_error(error)
            return False

    def set_motion_speed(self, speed:float) -> bool:
        """Set the speed slider on the TeachPendant. This controls the robot execution speed.
        
        TODO: Test if this respects MoveIt! configurations

        Args:
            speed (float): Speed factor of the arm. Will be clipped to range [0.0, 1.0].

        Raises:
            ROSException: When service call returned success = False.
            ServiceException: When service call failed.

        Returns:
            bool: True if speed slider (and execution speed) is set. False otherwise.
        """
        request = SetSpeedSliderFractionRequest()
        request.speed_slider_fraction = clip(speed, 0.0, 1.0)
        try:
            rospy.wait_for_service('/ur_hardware_interface/set_speed_slider', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/set_speed_slider', SetSpeedSliderFraction)(request)
            if response.success: 
                ur_log_success('Execution speed slider set')
                return response.success
            else:
                raise ROSException('Service call was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to set execution speed slider')
            ur_log_error(error)
            return False

    def zero_ft_sensor(self) -> bool:
        """zero the robot's ftsensor.

        Raises:
            ROSException: When service call returned success = False.
            ServiceException: When service call failed.

        Returns:
            bool: True if ftsensor is zeroed. False otherwise.
            
        Note: 
            Return value is success of trigger, not necessarily mean successful connection.
            Success usually mean the action is completed, assuming proper ROS connection.
        """
        try:
            rospy.wait_for_service('/ur_hardware_interface/zero_ftsensor', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/zero_ftsensor', Trigger)()
            if response.success:
                ur_log_success('Robot ftsensor is 0')
                return response.success
            else:
                raise ROSException('Service call was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to zero ftsensor')
            ur_log_error(error)
            return False

    # ---------------------------------------------------------------------------- #
    # ------------------------------ PROGRAM CONTROL ----------------------------- #
    # ---------------------------------------------------------------------------- #

    def load_installation(self, filename:str, wait:int = 10, reconnect_retries:int = 10) -> bool:
        """Load specified installation for the robot.

        Note:
            A known disconnection error will occur when swapping installation, which may be a part
            of loading a program or installation. This function will attempt to reconnect automatically.

        Args:
            filename (str): The installation file name with extension e.g. default.installation
            wait (int, optional): Wait time for installation to load properly. Defaults to 10.
            reconnect_retries (int, optional): Number of dashboard reconnect attempts. Defaults to 10.

        Returns:
            bool: True if installation is loaded properly, False otherwise.
        """
        request = LoadRequest()
        request.filename = filename
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/load_installation', timeout=self.service_timeout)
            rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_installation', Load)(request)
        except (ServiceException, ROSException) as _:
            ur_log_warn('Known dashboard server disconnection occured')
            ur_log_warn(f'Waiting for {wait} seconds for installation to load correctly')
            rospy.sleep(rospy.Duration(wait))
            ur_log_warn(f'Attempting to reconnect to dashboard server ({reconnect_retries} attempts)')
            self.spam_reconnect(reconnect_retries)
            self.close_popup()
        finally:
            self.last_known_installation = filename
            rospy.sleep(1)
            return True

    def load_program(self, filename:str, wait:int = 10, reconnect_retries:int = 10) -> bool:
        """Load specified program for the robot.

        Note:
            A known disconnection error will occur when swapping installation, which may be a part
            of loading a program or installation. This function will attempt to reconnect automatically.

        Args:
            filename (str): The program file name with extension e.g. default.program
            wait (int, optional): Wait time for program to load properly. Defaults to 10.
            reconnect_retries (int, optional): Number of dashboard reconnect attempts. Defaults to 10.

        Returns:
            bool: True if program is loaded properly, False otherwise.
        """
        request = LoadRequest()
        request.filename = filename
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/load_program', timeout=self.service_timeout)
            rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)(request)
        except (ServiceException, ROSException) as _:
            ur_log_warn('Known dashboard server disconnection occured')
            ur_log_warn(f'Waiting for {wait} seconds for program to load correctly')
            rospy.sleep(rospy.Duration(wait))
            ur_log_warn(f'Attempting to reconnect to dashboard server ({reconnect_retries} attempts)')
            self.spam_reconnect(reconnect_retries)
            self.close_popup()
        finally:
            self.last_known_program = filename
            rospy.sleep(1)
            return True
            
    def start_program(self) -> bool:
        """Start the loaded program.

        Raises:
            ROSException: When service call returned success = False.
            ServiceException: When service call failed.

        Returns:
            bool: True if program is now running. False otherwise.
            
        Note: 
            Return value is success of trigger, not necessarily mean successful connection.
            Success usually mean the action is completed, assuming proper ROS connection.
        """
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/play', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)()
            if response.success: 
                ur_log_success(f'Program {self.last_known_program} is now running')
                rospy.sleep(1)
                return response.success
            else:
                raise ROSException('Service call was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to start program')
            ur_log_error(error)
            return False
        
    def stop_program(self) -> bool:
        """Stop the loaded program.

        Raises:
            ROSException: When service call returned success = False.
            ServiceException: When service call failed.

        Returns:
            bool: True if program is now stopped. False otherwise.
            
        Note: 
            Return value is success of trigger, not necessarily mean successful connection.
            Success usually mean the action is completed, assuming proper ROS connection.
        """
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/stop', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger)()
            if response.success: 
                ur_log_success(f'Program {self.last_known_program} is now stopped')
                rospy.sleep(1)
                return response.success
            else:
                raise ROSException('Service call was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to stop program')
            ur_log_error(error)
            return False

    def pause_program(self):
        """Pause the loaded program.

        Raises:
            ROSException: When service call returned success = False.
            ServiceException: When service call failed.

        Returns:
            bool: True if program is now paused. False otherwise.
            
        Note: 
            Return value is success of trigger, not necessarily mean successful connection.
            Success usually mean the action is completed, assuming proper ROS connection.
        """
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/pause', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/pause', Trigger)()
            if response.success: 
                ur_log_success(f'Program {self.last_known_program} is now paused')
                rospy.sleep(1)
                return response.success
            else:
                raise ROSException('Service call was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to stop program')
            ur_log_error(error)
            return False
        
    def is_program_running(self) -> bool:
        """Query if the loaded program is running.

        Raises:
            ROSException: When service call returned success = False.
            ServiceException: When service call failed.

        Returns:
            bool: True if program is running. False otherwise.
        """
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/program_running', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_running', IsProgramRunning)()
            if response.success: 
                ur_log(f'Program {self.last_known_program} is {"" if response.program_running else "not"} running')
                return response.program_running
            else:
                raise ROSException('Service call was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to request program running state')
            ur_log_error(error)
            return False

    def is_program_saved(self) -> bool:
        """Query if the loaded program is running.

        Raises:
            ROSException: When service call returned success = False.
            ServiceException: When service call failed.

        Returns:
            bool: True if program is saved. False otherwise.
        """
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/program_saved', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_saved', IsProgramRunning)()
            if response.success: 
                ur_log(f'Program {self.last_known_program} is {"" if response.program_saved else "not"} saved')
                return response.program_saved
            else:
                raise ROSException('Service call was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to request program save state')
            ur_log_error(error)
            return False

    def get_program_state(self, suppressed:bool = False) -> tuple:
        """Get current program name and execution status.

        Args:
            suppressed (bool, optional): Do not output program name and status. Defaults to False.

        Raises:
            ROSException: When service call returned success = False.
            ServiceException: When service call failed.

        Returns:
            tuple: (program name, program state) or (None, None)
        """
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/program_state', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState)()
            if response.success: 
                self.last_known_program = response.program_name
                if not suppressed: ur_log_success(f'Program {self.last_known_program} is {response.state.state}')
                return (response.program_name, response.state.state)
            else:
                raise ROSException('Service call was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to request program state')
            ur_log_error(error)
            return (None, None)
        
    # only available in headless mode, not using
    def resend_robot_program(self):
        raise NotImplementedError
        
    # ---------------------------------------------------------------------------- #
    # ------------------------------- POPUP CONTROL ------------------------------ #
    # ---------------------------------------------------------------------------- #
    
    def close_popup(self, safety:bool = False) -> bool:
        """Close a popup on the TeachPendant.

        Args:
            safety (bool, optional): True when the targeted popup is a safety popup. Defaults to False.

        Raises:
            ROSException: When service call returned success = False.
            ServiceException: When service call failed. 

        Returns:
            bool: True if the popup is closed. False otherwise.
        """
        try:
            if safety:
                rospy.wait_for_service('/ur_hardware_interface/dashboard/close_safety_popup', timeout=self.service_timeout)
                response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/close_safety_popup', Trigger)()
            else:
                rospy.wait_for_service('/ur_hardware_interface/dashboard/close_popup', timeout=self.service_timeout)
                response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/close_popup', Trigger)()
            if response.success: 
                ur_log_success(f'{"Safety popup" if safety else "Popup"} closed')
                return response.success
            else:
                raise ROSException('Service call was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            ur_log_error(f'Unable to close {"safety" if safety else ""} popup')
            ur_log_error(error)
            return False

    def send_popup(self, message:str) -> bool:
        """Send a popup to the teach pendant.

        Args:
            message (str): Content of the popup.

        Raises:
            ROSException: When service call returned success = False.
            ServiceException: When service call failed.

        Returns:
            bool: True if the popup is sent successfully. False otherwise.
        """
        request = PopupRequest()
        request.message = message
        try:
            rospy.wait_for_service('/ur_hardware_interface/dashboard/popup', timeout=self.service_timeout)
            response = rospy.ServiceProxy('/ur_hardware_interface/dashboard/popup', Popup)(request)
            if response.success: 
                ur_log_success('Popup sent to pendant')
                return response.success
            else:
                raise ROSException('Service call was not successful (response.success = False)') 
        except (ServiceException, ROSException) as error:
            ur_log_error('Unable to send popup')
            ur_log_error(error)
            return False
    
    # ---------------------------------------------------------------------------- #
    # -------------------------------- I/O CONTROL ------------------------------- #
    # ---------------------------------------------------------------------------- #

    #TODO: add strict control (if/else) of input to match configurations
    def set_io(self, function:int, pin:int, state:int) -> None:
        """Set I/O state on the control box.

        Args:
            function (int): Type of operation to perform.
            pin (int): Which pin to perform the operation on (may be ignored).
            state (int): Resulting state of the pin.
        """
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