#!/usr/bin/env python3

from enum import Enum

from robotiq_ros_wrapper.msg import Robotiq2FInput, Robotiq2FOutput

from numpy import poly1d, polyfit, clip
from math import degrees, radians

def binary_to_rad(rad, limit = (0, 255), stroke = 85):
    return poly1d(polyfit(limit, (0, (0.8 if stroke == 85 else 0.7)), 1))(rad)

class robotiq_force(object):
    def __init__(self, stroke = 85, precision = 2):
        self._N   = 0.0
        self._lbf = 0.0
        self._raw = 0
        self._limit  = (0, 255)
        self._stroke = stroke
        self._precision = precision
        
        self._N_range   = ( 20,  235) if self._stroke == 85 else ( 10,  125)
        self._lbf_range = (4.5, 52.8) if self._stroke == 85 else (2.2, 28.1)
        
        self._raw_to_N = poly1d(polyfit(self._limit, self._N_range, 1))
        self._N_to_raw = poly1d(polyfit(self._N_range, self._limit, 1))
    
    @property
    def limit(self):
        return self._limit
        
    @property
    def raw(self):
        return self._raw
    
    @property
    def N(self):
        return self._N
    
    @property
    def lbf(self):
        return self._lbf
    
    @limit.setter
    def limit(self, value:tuple):
        if len(value) >= 2 and value[0] < value[1]:
            if 0 <= value[0] <= 255 and 0 <= value[1] <= 255:
                self._limit = (value[0], value[1])
    
    @raw.setter
    def raw(self, value):
        self._raw = int(clip(value, *self._limit))
        self._N = self._raw_to_N(self._raw)
        self._lbf = self._N * 0.224809
    
    @N.setter
    def N(self, value):
        self._N = clip(value, *self._N_range)
        self._lbf = self._N * 0.224809
        self._raw = int(self._N_to_raw(value))
        
    @lbf.setter
    def lbf(self, value):
        self._lbf = clip(value, *self._lbf_range)
        self._N = self.lbf / 0.224809
        self._raw = int(self._N_to_raw(value))
        
    def print_all(self):
        print(f'limit = {self._limit}')
        print(f'raw = {self._raw}')
        print(f'  N = {self._N}')
        print(f'lbf = {self._lbf}')
        print()

class robotiq_speed(object):
    def __init__(self, stroke = 85, precision = 2):
        self._mm_s = 0.0
        self._in_s = 0.0
        self._raw  = 0
        self._limit  = (0, 255)
        self._stroke = stroke
        self._precision = precision
        
        self._mm_range = ( 20, 150) if self._stroke == 85 else ( 30, 250)
        self._in_range = (0.8, 5.9) if self._stroke == 85 else (1.2, 9.8)
        
        self._mm_s_to_raw = poly1d(polyfit(self._mm_range, self._limit, 1))
        self._raw_to_mm_s = poly1d(polyfit(self._limit, self._mm_range, 1))
    
    @property
    def limit(self):
        return self._limit
    
    @property
    def raw(self):
        return self._raw
    
    @property
    def mmps(self):
        return self._mm_s
    
    @property
    def inps(self):
        return self._in_s
        
    @limit.setter
    def limit(self, value:tuple):
        if len(value) >= 2 and value[0] < value[1]:
            if 0 <= value[0] <= 255 and 0 <= value[1] <= 255:
                self._limit = (value[0], value[1])
        
    @raw.setter
    def raw(self, value):
        self._raw = int(clip(value, *self._limit))
        self._mm_s = self._raw_to_mm_s(self._raw)
        self._in_s = self._mm_s * 0.0393701
    
    @mmps.setter
    def mmps(self, value):
        self._mm_s = clip(value, *self._mm_range)
        self._in_s = self._mm_s * 0.0393701
        self._raw = int(self._mm_s_to_raw(self._mm_s))
    
    @inps.setter
    def inps(self, value):
        self._in_s = clip(value, *self._in_range)
        self._mm_s = self._in_s / 0.0393701
        self._raw = int(self._mm_s_to_raw(self._mm_s))
        
    def print_all(self):
        print(f'limit = {self._limit}')
        print(f'raw  = {self._raw}')
        print(f'mm/s = {self._mm_s}')
        print(f'in/s = {self._in_s}')
        print()
    
class robotiq_distance(object):
    def __init__(self, limit = (0, 255), stroke = 85, precision = 2):
        self._deg = 0.0
        self._rad = 0.0
        self._jaw_in = 0.0
        self._jaw_mm = 0.0
        self._raw = 0
        self._limit  = limit
        self._stroke = stroke
        self._precision = precision
        
        self._jaw_range_mm = (0, self._stroke)
        self._jaw_range_in = (0, self._stroke / 25.4)
        self._rad_range = (0.0, 0.8 if self._stroke == 85 else 0.7)
        self._deg_range = (0.0, degrees(0.8 if self._stroke == 85 else 0.7))
        
        self._experiment_raw = [100, 120, 140, 160, 180]
        self._experiment_act = [50.3, 42.05, 33.76, 25.5, 17.23]
        
        # self._raw_to_mm = poly1d(polyfit(self._limit, self._jaw_range_mm, 1))
        self._raw_to_mm = poly1d(polyfit(self._experiment_raw, self._experiment_act, 1))
        self._mm_to_raw = poly1d(polyfit(self._experiment_act, self._experiment_raw, 1))
        self._raw_to_rad = poly1d(polyfit(self._limit, self._rad_range, 1))
        self._rad_to_raw = poly1d(polyfit(self._rad_range, self._limit, 1))
        
    @property
    def limit(self):
        return self._limit
    
    @property
    def raw(self):
        return self._raw
    
    @property
    def rad(self):
        return self._rad
    
    @property
    def deg(self):
        return self._deg
    
    @property
    def opening_in(self):
        return self._jaw_in
    
    @property
    def opening_mm(self):
        return self._jaw_mm
    
    @limit.setter
    def limit(self, value:tuple):
        if len(value) >= 2 and value[0] < value[1]:
            if 0 <= value[0] <= 255 and 0 <= value[1] <= 255:
                self._limit = (value[0], value[1])
                
    @raw.setter
    def raw(self, value):
        self._raw = int(clip(value, *self._limit))
        self._jaw_mm = clip(self._raw_to_mm(self._raw), *self._jaw_range_mm)
        self._jaw_in = self._jaw_mm / 25.4
        self._rad = self._raw_to_rad(self._raw)
        self._deg = degrees(self._rad)
        
    @deg.setter
    def deg(self, value):
        self._deg = clip(value, *self._deg_range)
        self._rad = radians(self._deg)
        self._raw = int(self._rad_to_raw(self._rad))
        self._jaw_mm = clip(self._raw_to_mm(self._raw), *self._jaw_range_mm)
        self._jaw_in = self._jaw_mm / 25.4
        
    @rad.setter
    def rad(self, value):
        self._rad = clip(value, *self._rad_range)
        self._deg = degrees(self._rad)
        self._raw = int(self._rad_to_raw(self._rad))
        self._jaw_mm = clip(self._raw_to_mm(self._raw), *self._jaw_range_mm)
        self._jaw_in = self._jaw_mm / 25.4
        
    @opening_mm.setter
    def opening_mm(self, value):
        self._jaw_mm = clip(value, *self._jaw_range_mm)
        self._jaw_in = self._jaw_mm / 25.4
        self._raw = int(self._mm_to_raw(self._jaw_mm))
        self._rad = self._raw_to_rad(self._raw)
        self._deg = degrees(self._rad)
        
    @opening_in.setter
    def opening_in(self, value):
        self._jaw_in = clip(value, *self._jaw_range_in)
        self._jaw_mm = self._jaw_mm * 25.4
        self._raw = int(self._mm_to_raw(self._jaw_mm))
        self._rad = self._raw_to_rad(self._raw)
        self._deg = degrees(self._rad)
        
    def print_all(self):
        print(f'raw     = {self._raw}')
        print(f'rad     = {self._rad}')
        print(f'degrees = {self._deg}')
        print(f'gap mm  = {self._jaw_mm}')
        print(f'gap in  = {self._jaw_in}')
        print()

def gen_2f_msg_from_bin(binary_msg:list) -> Robotiq2FInput:
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

def gen_bin_from_2f_msg(output_msg:Robotiq2FOutput) -> list:
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