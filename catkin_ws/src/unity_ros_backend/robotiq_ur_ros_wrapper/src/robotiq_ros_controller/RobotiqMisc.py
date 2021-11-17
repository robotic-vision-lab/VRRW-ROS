#!/usr/bin/env python3

from enum import Enum

from numpy import clip, interp
from math import radians, degrees

from robotiq_ur_ros_wrapper.msg import Robotiq2FInput, Robotiq2FOutput

from ur_remote_dashboard.URMisc import PrintColor

rbt_pcolor = PrintColor()

def rbt_raw_to_rad(raw):
    return interp(clip(raw, 0.0, 255.0), [0.0, 255.0], [0.0, 0.8])

def rbt_rad_to_raw(rad):
    return interp(clip(rad, 0.0, 0.8), [0.0, 0.8], [0.0, 255.0])

def generate_input_message_from_binary(binary_msg:list) -> Robotiq2FInput:
    formatted = Robotiq2FInput()
    formatted.activated         = (binary_msg[0] >> 0) & 0x01;
    formatted.action_status     = (binary_msg[0] >> 3) & 0x01;
    formatted.gripper_status    = (binary_msg[0] >> 4) & 0x03;
    formatted.object_status     = (binary_msg[0] >> 6) & 0x03;
    formatted.fault_status      =  binary_msg[2]
    formatted.position_request  =  binary_msg[3]
    formatted.current_position  =  binary_msg[4]
    formatted.motor_current     =  binary_msg[5]
    return formatted

def generate_binary_from_output_message(output_msg:Robotiq2FOutput) -> list:
    bin_cmd = []
    bin_cmd.append(output_msg.activate + (output_msg.goto << 3) + (output_msg.trigger_autorelease << 4))
    bin_cmd.append(0)
    bin_cmd.append(0)
    bin_cmd.append(output_msg.position)
    bin_cmd.append(output_msg.speed)
    bin_cmd.append(output_msg.force)
    return bin_cmd

def rbt_log_success(msg):
    print(f'{rbt_pcolor.flags["SUCCESS"]}[ROBOTIQ] {msg}{rbt_pcolor.effects["RESET"]}')
    
def log_error(msg):
    print(f'{rbt_pcolor.flags["ERROR"]}[ROBOTIQ] {msg}{rbt_pcolor.effects["RESET"]}')

def rbt_log_success(msg):
    print(f'{rbt_pcolor.flags["SUCCESS"]}[ROBOTIQ] {msg}{rbt_pcolor.effects["RESET"]}')

def rbt_log(msg):
    print(f'{rbt_pcolor.flags["LOG"]}[ROBOTIQ] {msg}{rbt_pcolor.effects["RESET"]}')

def rbt_warn(msg):
    print(f'{rbt_pcolor.flags["WARNING"]}[ROBOTIQ] {msg}{rbt_pcolor.effects["RESET"]}')
    
class Robotiq2FStatusMapping(Enum):
    pass