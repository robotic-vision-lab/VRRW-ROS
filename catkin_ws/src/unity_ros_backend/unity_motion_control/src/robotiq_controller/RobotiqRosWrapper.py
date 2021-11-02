#!/usr/bin/env python3

import rospy

from robotiq_controller.RobotiqModbusServer import RobotiqRTUClient

from unity_motion_control.msg import Robotiq2FInput, Robotiq2FOutput

from sensor_msgs.msg import JointState

device_name = rospy.get_param('/ur_tool_communication_bridge/device_name')

def generate_ros_msg(self, raw_status):
    pass

def start_robotiq_backend():
    gripper_interface = RobotiqRTUClient()
    gripper_interface.connect(device_name)
    
    gripper_status_publisher = rospy.Publisher('robotiq_gripper_status', Robotiq2FInput, queue_size=1)
    gripper_joint_state_publisher = rospy.Publisher('robotiq_joint_publisher', JointState, queue_size=1)
    
    while not rospy.is_shutdown():
        new_status = gripper_interface.request_status()
        
    
if __name__ == '__main__':
    try:
        start_robotiq_backend()
    except rospy.ROSInterruptException: 
        pass