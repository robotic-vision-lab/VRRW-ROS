#!/usr/bin/env python3

import rospy

from robotiq_modbus_server.RobotiqModbusServer import RobotiqRTUClient
from robotiq_ur_ros_wrapper.msg import Robotiq2FInput, Robotiq2FOutput

from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from robotiq_ros_controller.RobotiqMisc import *

def send_command_to_gripper(msg:Robotiq2FOutput, communicator:RobotiqRTUClient) -> None:
    communicator.send_command(generate_binary_from_output_message(msg))
    
def publish_gripper_joint_state(msg:Robotiq2FInput, publisher:rospy.Publisher):
    gripper_joint_state = JointState()
    gripper_joint_state.header = Header()
    gripper_joint_state.header.stamp = rospy.Time.now()
    gripper_joint_state.name = ['finger_joint']
    gripper_joint_state.position = [rbt_raw_to_rad(msg.current_position)]
    gripper_joint_state.effort = [0.0]
    gripper_joint_state.velocity = [0.0]
    
    publisher.publish(gripper_joint_state)

def start_robotiq_rs485_backend(device_name, refresh_rate=200):
    gripper = RobotiqRTUClient()
    gripper.connect(device_name)
    limiter = rospy.Rate(refresh_rate)
    
    joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=1)
    
    status_publisher = rospy.Publisher('robotiq_gripper_status', Robotiq2FInput, queue_size=1)
    status_monitor = rospy.Subscriber('robotiq_gripper_status', Robotiq2FInput, callback=lambda msg:publish_gripper_joint_state(msg, joint_state_publisher))

    command_publisher = rospy.Publisher('robotiq_gripper_command_interface', Robotiq2FOutput, queue_size=1)
    command_monitor = rospy.Subscriber('robotiq_gripper_command_interface', Robotiq2FOutput, callback=lambda msg: send_command_to_gripper(msg, gripper))
    
    rospy.logwarn(f'Waiting for {5} seconds so publishers and subscribers register correctly')
    rospy.sleep(5)
    rbt_log_success('Robotiq 2-Finger Gripper ready to take command')
    
    while not rospy.is_shutdown():
        status_publisher.publish(generate_input_message_from_binary(gripper.request_status()))
        limiter.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('robotiq_interface_node', anonymous=True)
        start_robotiq_rs485_backend(device_name=rospy.get_param('/ur_tool_communication_bridge/device_name'))
    except Exception as e:
        print(e)
        pass 