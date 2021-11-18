#!/usr/bin/env python3

from enum import Enum

from numpy import clip, interp
from math import radians, degrees

from robotiq_ros_wrapper.msg import Robotiq2FInput, Robotiq2FOutput

def rbt_raw_to_rad(raw):
    return interp(clip(raw, 0.0, 255.0), [0.0, 255.0], [0.0, 0.8])

def rbt_rad_to_raw(rad):
    return interp(clip(rad, 0.0, 0.8), [0.0, 0.8], [0.0, 255.0])

def generate_input_message_from_binary(binary_msg:list) -> Robotiq2FInput:
    formatted = Robotiq2FInput()
    formatted.activated         = (binary_msg[0] >> 0) & 0x01
    formatted.action_status     = (binary_msg[0] >> 3) & 0x01
    formatted.gripper_status    = (binary_msg[0] >> 4) & 0x03
    formatted.object_status     = (binary_msg[0] >> 6) & 0x03
    formatted.fault_status      =  binary_msg[2]
    formatted.position_request  =  binary_msg[3]
    formatted.current_position  =  binary_msg[4]
    formatted.motor_current     =  binary_msg[5]
    return formatted

def generate_binary_from_output_message(output_msg:Robotiq2FOutput) -> list:
    bin_cmd = []
    
    # byte 0: action request
    bin_cmd.append(output_msg.activate + (output_msg.goto << 3) + (output_msg.trigger_autorelease << 4) + (output_msg.autorelease_direction << 5))
    
    # byte 1 and 2 (reserved)
    bin_cmd.append(0)
    bin_cmd.append(0)
    
    # byte 3: position request
    bin_cmd.append(output_msg.position)
    
    # byte 4: speed
    bin_cmd.append(output_msg.speed)
    
    # byte 5: force
    bin_cmd.append(output_msg.force)
    
    return bin_cmd