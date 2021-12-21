#!/usr/bin/env python3

# ROS packages
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

# Robotiq
from robotiq_controller.RobotiqModbusServer import RobotiqRTUClient
from ur_remote_control.msg import Robotiq2FCommand, Robotiq2FStatus

def start_robotiq_backend(device_name, refresh_rate=200):
    pass

if __name__ == '__main__':
    try:
        rospy.init_node('robotiq_interface_node', anonymous=True)
        gripper_addr = rospy.get_param('/ur_tool_communication_bridge/device_name', default='/tmp/ttyUR')
        start_robotiq_backend(device_name=gripper_addr)
    except Exception as e:
        print(e)
        pass