#!/usr/bin/env python3

from logging import ERROR
from numpy import clip, interp
from enum import Enum

import rospy

from backend_support.RobotiqModbusServer import RobotiqRTUClient

from unity_planning_backend.msg import Robotiq2FInput, Robotiq2FOutput

class Robotiq2FController:
    def __init__(self, device='/tmp/ttyUR', autoconnect=False):
        self.communicator = RobotiqRTUClient()
        self.device = device
        
        # raw binary status
        self.activation_status = 0
        self.action_status = 0
        self.gripper_status = 0
        self.object_status = 0
        self.fault_status = 0
        self.position_request = 0
        self.current_position = 0
        self.motor_current = 0
        
        # human readable (for printing - not used internally)
        # assume cold-boot, fully open state
        self.reset = True
        self.ready = False
        self.moving = False
        self.holding = False
        self.fault = 'No Fault'
        self.position = [0, 0]
        
        if autoconnect:
            self.connect()
            self.update_status()
            
    def connect(self):
        self.communicator.connect(device=self.device)
    
    def update_status(self):
        raw_status = self.communicator.request_status()
        
        # decode raw registers return
        self.activation_status = (raw_status[0] >> 0) & 0x01;
        self.action_status     = (raw_status[0] >> 3) & 0x01;
        self.gripper_status    = (raw_status[0] >> 4) & 0x03;
        self.object_status     = (raw_status[0] >> 6) & 0x03;
        self.fault_status      =  raw_status[2]
        self.position_request  =  raw_status[3]
        self.current_position  =  raw_status[4]
        self.motor_current     =  raw_status[5]
        
        self.reset = True if (self.activation_status == 0 or self.gripper_status == 0) else False
        self.ready = True if (self.activation_status == 1 or self.gripper_status == 3) else False
        
        if self.action_status == 1 and self.object_status == 0:
            self.moving = True
        elif self.object_status != 0:
            self.moving = False
            
        if self.object_status == 0:
            self.holding = (False, 'Moving to requested position')
        elif self.object_status == 1:
            self.holding = (True, 'Holding on opening')
        elif self.object_status == 2:
            self.holding = (True, 'Holding on closing')
        elif self.object_status == 3:
            self.holding = (False, 'At requested position, not holding/lost object')
        else:
            self.holding = (None, 'WARNING! NOT SUPPOSED TO HAPPEN!')
            
        # TODO: FAULT REPORTING
        
        # TODO: POSITION REPORTING
            
    def print_status(self, report_raw=False):
        self.update_status()
        print('=' * 10 + ' Current status')
        if report_raw:
            print(f'  ----- Raw status -----')
            print(f'       Activation Status (gACT) = {self.activation_status}')
            print(f'           Action Status (gGTO) = {self.action_status}')
            print(f'          Gripper Status (gSTA) = {self.gripper_status}')
            print(f'Object Dectection Status (gOBJ) = {self.object_status}')
            print(f'            Fault Status (gFLT) = {self.fault_status}')
            print(f'        Position Request (gPR)  = {self.position_request}')
            print(f'        Current Position (gPO)  = {self.current_position}')
            print(f'           Motor Current (gCU)  = {self.motor_current}')
        print(f'    RESET = {self.reset}')
        print(f'    READY = {self.ready}')
        print(f'   MOVING = {self.moving}')
        print(f'  HOLDING = {self.holding[0]} ({self.holding[1]})')
        
    def generate_command(self, activate = 1, goto = 0, trigger_autorelease = 0, position = 0, speed = 0, force = 0):
        command = []
        command.append(activate + (goto << 3) + (trigger_autorelease << 4))
        command.append(0)
        command.append(0)
        command.append(position)
        command.append(speed)
        command.append(force)
        
        return command
        
    def reset_gripper(self):
        self.communicator.send_command(self.generate_command(activate=0))
    
    def activate_gripper(self):
        self.communicator.send_command(self.generate_command(goto=1, position=0, speed=255, force=150))
        
    def autoclose(self, force = 150):
        self.communicator.send_command(self.generate_command(goto=1, position=255, speed=255, force=force))
    
    def autoopen(self, force = 150):
        self.communicator.send_command(self.generate_command(goto=1, position=0, speed=255, force=force))
        
    def conv_vel_percent(self, vel):
        if vel > 1.0: vel /= 1.0
        return int(vel * 255.0)
    
    def conv_vel_real(self, vel):
        return int(interp(clip(vel, 20.0, 150.0), [0.0, 150.0], [0.0, 255.0]))
    
    def conv_pos_percent(self, pos):
        if pos > 1.0: pos /= 1.0
        return int(pos * 255.0)
    
    def conv_pos_real(self, pos):
        return int(interp(clip(pos, 0.0, 0.0085), [0.0, 0.0085], [0.0, 255.0]))
    
    def conv_effort_percent(self, N):
        if N > 1.0: N /= 1.0
        return int(N * 255.0)
    
    def conv_effort_real(self, N):
        return interp(clip(N, 20.0, 235.0), [20.0, 235.0], [0.0, 255.0])
    
    def conv_raw_pos(self, raw):
        return interp(clip(raw, 0.0, 255.0), [0.0, 255.0], [0.0, 0.0085])
    
    def conv_raw_vel(self, raw):
        return interp(clip(raw, 0.0, 255.0), [0.0, 255.0], [0.0020, 0.0085])
    
    def conv_raw_effor(self, raw):
        return interp(clip(raw, 0.0, 255.0), [0.0, 255.0], [20.0, 235.0])
    
class Robotiq2FControllerNode:
    def __init__(self):
        raise NotImplementedError