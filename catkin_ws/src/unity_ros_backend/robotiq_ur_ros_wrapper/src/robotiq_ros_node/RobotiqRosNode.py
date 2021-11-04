#!/usr/bin/env python3

import rospy

from robotiq_modbus_server.RobotiqModbusServer import RobotiqRTUClient
from robotiq_ur_ros_wrapper.msg import Robotiq2FInput, Robotiq2FOutput

def print_stamping(msg):
    print(f'[ROBOTIQ ROS NODE] {msg}')

def generate_robotiq_ros_msg(raw_msg):
    formatted = Robotiq2FInput()
    formatted.activated         = (raw_msg[0] >> 0) & 0x01;
    formatted.action_status     = (raw_msg[0] >> 3) & 0x01;
    formatted.gripper_status    = (raw_msg[0] >> 4) & 0x03;
    formatted.object_status     = (raw_msg[0] >> 6) & 0x03;
    formatted.fault_status      =  raw_msg[2]
    formatted.position_request  =  raw_msg[3]
    formatted.current_position  =  raw_msg[4]
    formatted.motor_current     =  raw_msg[5]
    return formatted
    
def generate_binary_command(command):
    bin_cmd = []
    bin_cmd.append(command.activate + (command.goto << 3) + (command.trigger_autorelease << 4))
    bin_cmd.append(0)
    bin_cmd.append(0)
    bin_cmd.append(command.position)
    bin_cmd.append(command.speed)
    bin_cmd.append(command.force)
    return bin_cmd
    
def send_command_to_gripper(msg, communicator):
    command = generate_binary_command(command=msg)
    communicator.send_command(command)

def start_robotiq_rs485_backend(device_name, refresh_rate=200):
    gripper = RobotiqRTUClient()
    gripper.connect(device_name)
    limiter = rospy.Rate(refresh_rate)
    
    status_publisher = rospy.Publisher('robotiq_gripper_status', Robotiq2FInput, queue_size=1)
    command_publisher = rospy.Publisher('robotiq_gripper_command_interface', Robotiq2FOutput, queue_size=1)
    command_monitor = rospy.Subscriber('robotiq_gripper_command_interface', Robotiq2FOutput, callback=lambda msg: send_command_to_gripper(msg, gripper))
    
    while not rospy.is_shutdown():
        status_publisher.publish(generate_robotiq_ros_msg(gripper.request_status()))
        limiter.sleep()

if __name__ == '__main__':
    default_wait = 10
    try:
        rospy.init_node('robotiq_interface_node', anonymous=True)
        print_stamping(f'Waiting for {default_wait} all other ROS services to come up')
        rospy.sleep(rospy.Duration(default_wait))
        print_stamping(f'Starting Robotiq status messages...')
        start_robotiq_rs485_backend(device_name=rospy.get_param('/ur_tool_communication_bridge/device_name'))
    except Exception as e:
        print(e)
        pass 