#!/usr/bin/env python3

from enum import Enum

class PrintColor():
    def __init__(self):
        self.effects = {
            'RESET'     : self.gen_esc(0),
            'BOLD'      : self.gen_esc(1),
            'UNDERLINE' : self.gen_esc(4)
        }
        
        self.default_colors = {
            'RED'       : self.gen_esc_ansi_rgb(self.hex_to_rgb('#ff0000')),
            'GREEN'     : self.gen_esc_ansi_rgb(self.hex_to_rgb('#00ff00')),
            'BLUE'      : self.gen_esc_ansi_rgb(self.hex_to_rgb('#0000ff')),
            'CYAN'      : self.gen_esc_ansi_rgb(self.hex_to_rgb('#00ffff')),
            'MAGENTA'   : self.gen_esc_ansi_rgb(self.hex_to_rgb('#ff00ff')),
            'YELLOW'    : self.gen_esc_ansi_rgb(self.hex_to_rgb('#ffff00')),
            'WHITE'     : self.gen_esc_ansi_rgb(self.hex_to_rgb('#ffffff')),
            'BLACK'     : self.gen_esc_ansi_rgb(self.hex_to_rgb('#000000')),
        }
        
        self.dracula_colors = {
            'CYAN'      : self.gen_esc_ansi_rgb(self.hex_to_rgb('#8be9fd')),
            'GREEN'     : self.gen_esc_ansi_rgb(self.hex_to_rgb('#50fa7b')),
            'ORANGE'    : self.gen_esc_ansi_rgb(self.hex_to_rgb('#ffb86c')),
            'PINK'      : self.gen_esc_ansi_rgb(self.hex_to_rgb('#ff79c6')),
            'PURPLE'    : self.gen_esc_ansi_rgb(self.hex_to_rgb('#bd93f9')),
            'RED'       : self.gen_esc_ansi_rgb(self.hex_to_rgb('#ff5555')),
            'YELLOW'    : self.gen_esc_ansi_rgb(self.hex_to_rgb('#f1fa8c'))
        }
        
        self.flags = {
            'ERROR'     : self.gen_esc(91),
            'SUCCESS'   : self.gen_esc(92),
            'WARNING'   : self.gen_esc(93),
            'LOG'       : self.default_colors['WHITE']
        }
    
    def color_test(self):
        for name, code in self.flags.items():
            print(f'{name:<10}{code} test message{self.effects["RESET"]}')
        
    def gen_esc(self, code):
        return f"\033[{code}m"
    
    def gen_esc_ansi_rgb(self, code):
        return f"\033[38;2;{code}m"
    
    def hex_to_rgb(self, hex, ansi=True):
        if hex[0] == '#': hex = hex[1:]
        rgb = tuple(int(hex[i:i+2], 16) for i in (0, 2, 4))
        if ansi:
            return f'{rgb[0]};{rgb[1]};{rgb[2]}'
        else:
            return rgb
        
    def rgb_to_hex(self, rgb):
        return f'#{rgb[0]:x}{rgb[1]:x}{rgb[2]:x}'.upper()
    
# p = PrintColor()
# p.color_test()

class RobotModeMapping(Enum):
    NO_CONTROLLER=-1
    DISCONNECTED=0
    CONFIRM_SAFETY=1
    BOOTING=2
    POWER_OFF=3
    POWER_ON=4
    IDLE=5
    BACKDRIVE=6
    RUNNING=7
    UPDATING_FIRMWARE=8

class SafetyModeMapping(Enum):
    NORMAL=1
    REDUCED=2
    PROTECTIVE_STOP=3
    RECOVERY=4
    SAFEGUARD_STOP=5
    SYSTEM_EMERGENCY_STOP=6
    ROBOT_EMERGENCY_STOP=7
    VIOLATION=8
    FAULT=9
    VALIDATE_JOINT_ID=10
    UNDEFINED_SAFETY_MODE=11
    AUTOMATIC_MODE_SAFEGUARD_STOP=12
    SYSTEM_THREE_POSITION_ENABLING_STOP=13
    
class SetIOFunction(Enum):
    # FUN_SET_DIGITAL_OUT = 1
    # FUN_SET_FLAG = 2
    # FUN_SET_ANALOG_OUT = 3
    # FUN_SET_TOOL_VOLTAGE = 4
    
    SET_DIGITAL_OUT = 1
    SET_FLAG = 2
    SET_ANALOG_OUT = 3
    SET_TOOL_VOLTAGE = 4
    
class SetIOPinState(Enum):
    OFF = 0
    ON = 1
    
class SetIOToolState(Enum):
    TOOL_VOLTAGE_0V = 0
    TOOL_VOLTAGE_12V = 12
    TOOL_VOLTAGE_24V = 24
    
class SetIOPinMapping(Enum):
    pass