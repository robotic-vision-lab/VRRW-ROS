#!/usr/bin/env python3

# ROS packages
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

# Robotiq
from robotiq_controller.RobotiqModbusServer import RobotiqRTUClient
from robotiq_controller.Robotiq2FSupport import *
from ur_remote_control.msg import Robotiq2FCommand, Robotiq2FStatus

def start_robotiq_backend(device, refresh_rate = 200, bypass = False):
    gripper = RobotiqRTUClient()
    gripper.connect(device)
    limiter = rospy.Rate(hz = refresh_rate)

    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('robotiq_interface_node', anonymous=True)
        start_robotiq_backend(device=rospy.get_param('/ur_tool_communication_bridge/device_name', default='/tmp/ttyUR'))
    except Exception as e:
        rospy.logerr(e)