from rvl_robotiq_controller.msg import Robotiq2FStatus, Robotiq2FCommand
from rvl_utilities.CustomLogger import ColorLogger

from numpy import poly1d, polyfit, clip

r2f85_conv  = poly1d(polyfit((0, 255), (0, 0.8), 1))
r2f140_conv = poly1d(polyfit((0, 255), (0, 0.7), 1))

def raw_to_rad_2f85(raw):
    return r2f85_conv(int(clip(raw, 0.0, 255.0)))

def raw_to_rad_2f140(raw):
    return r2f140_conv(int(clip(raw, 0.0, 255.0)))

def generate_2f_status_from_binary(binary):
    status = Robotiq2FStatus()
    status.activated         = (binary[0] >> 0) & 0x01
    status.action_status     = (binary[0] >> 3) & 0x01
    status.gripper_status    = (binary[0] >> 4) & 0x03
    status.object_status     = (binary[0] >> 6) & 0x03
    status.fault_status      =  binary[2]
    status.position_request  =  binary[3]
    status.current_position  =  binary[4]
    status.motor_current     =  binary[5]
    return status

def generate_binary_command_from_2f_msg(message):
    command = []

    # byte 0: action request
    command.append(message.activate + (message.goto << 3) + (message.trigger_autorelease << 4) + (message.autorelease_direction << 5))

    # byte 1 and 2 (reserved)
    command.append(0)
    command.append(0)

    # byte 3: position request
    command.append(message.position)

    # byte 4: speed
    command.append(message.speed)

    # byte 5: force
    command.append(message.force)

    return command