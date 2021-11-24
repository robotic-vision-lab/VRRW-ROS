#!/usr/bin/env python3

import rospy

from robotiq_modbus_server.RobotiqModbusServer import RobotiqRTUClient
from robotiq_ros_wrapper.msg import Robotiq2FInput, Robotiq2FOutput

from robotiq_ros_controller.RobotiqMisc import rbt_log_error, rbt_log_success

from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from robotiq_ros_controller.Robotiq2FSupport import *

param_status_topic = rospy.get_param('/robotiq_2f_controller/status_topic', default='/Robotiq2F/gripper_status')
param_command_topic = rospy.get_param('/robotiq_2f_controller/command_topic', default='/Robotiq2F/command_interface')
# param_limit = tuple(rospy.get_param('robotiq_2f_controller/raw_range', default=[0, 255]))
param_stroke = rospy.get_param('robotiq_2f_controller/stroke', default=85)

def send_command_to_gripper(msg:Robotiq2FOutput, communicator:RobotiqRTUClient) -> None:
    communicator.send_command(gen_bin_from_2f_msg(msg))
    
def publish_gripper_joint_state(msg:Robotiq2FInput, publisher:rospy.Publisher):
    gripper_joint_state = JointState()
    gripper_joint_state.header = Header()
    gripper_joint_state.header.stamp = rospy.Time.now()
    gripper_joint_state.name = ['finger_joint']
    gripper_joint_state.position = [binary_to_rad(msg.current_position, limit=tuple(rospy.get_param('robotiq_2f_controller/raw_range', default=[0, 255])), stroke=param_stroke)]
    gripper_joint_state.effort = [0.0]
    gripper_joint_state.velocity = [0.0]
    
    publisher.publish(gripper_joint_state)

def start_robotiq_rs485_backend(device_name, refresh_rate=200):
    gripper = RobotiqRTUClient()
    gripper.connect(device_name)
    limiter = rospy.Rate(refresh_rate)
    
    joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=1)
    
    status_publisher = rospy.Publisher(param_status_topic, Robotiq2FInput, queue_size=200)
    status_monitor = rospy.Subscriber(param_status_topic, Robotiq2FInput, callback=lambda msg:publish_gripper_joint_state(msg, joint_state_publisher))

    command_publisher = rospy.Publisher(param_command_topic, Robotiq2FOutput, queue_size=10)
    command_monitor = rospy.Subscriber(param_command_topic, Robotiq2FOutput, callback=lambda msg: send_command_to_gripper(msg, gripper))
    
    rospy.logwarn(f'Waiting for {5} seconds so publishers and subscribers register correctly')
    rospy.sleep(5)
    rbt_log_success('Robotiq 2F Gripper ready to take commands')
    
    while not rospy.is_shutdown():
        status_publisher.publish(gen_2f_msg_from_bin(gripper.request_status()))
        limiter.sleep()
        
    gripper.disconnect()
    
if __name__ == '__main__':
    try:
        rospy.init_node('robotiq_interface_node', anonymous=True)
        start_robotiq_rs485_backend(device_name=rospy.get_param('/ur_tool_communication_bridge/device_name', default='/tmp/ttyUR'))
    except Exception as e:
        print(e)
        pass 