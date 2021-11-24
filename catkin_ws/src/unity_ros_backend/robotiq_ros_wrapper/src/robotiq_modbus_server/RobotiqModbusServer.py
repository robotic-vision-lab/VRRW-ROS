#!/usr/bin/env python3

import rospy

from pymodbus.client.sync import ModbusSerialClient, ModbusTcpClient
from pymodbus.register_read_message import ReadHoldingRegistersResponse
from math import ceil

class RobotiqRTUClient:
    def __init__(self, slave_id=0x0009):
        self.client = None
        
        # Robotiq manual page 67
        self.slave_id = slave_id
        self.input_register = 0x03E8
        self.output_register = 0x07D0
        
        # https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/ROS_INTERFACE.md#tool_device_name-default-tmpttyur-2
        # self.device = device
        
    def connect(self, device, lag = 1):
        self.client = ModbusSerialClient(method='rtu',port=device,stopbits=1, bytesize=8, baudrate=115200, timeout=0.2)
        while not self.client.connect() and not rospy.is_shutdown():
            rospy.logwarn_once(f'Unable to connect to {device}, trying every {lag} seconds in background...')
            rospy.sleep(lag)
    
    def disconnect(self):
        self.client.close()

    def send_command(self, command):
        # make sure data has an even number of elements
        if(len(command) % 2 == 1):
            command.append(0)

        # initiate message as an empty list
        message = []

        # fill message by combining two bytes in one register
        for i in range(0, len(command)//2):
            message.append((command[2*i] << 8) + command[2*i+1])
            
        # sending the command
        # TODO: implement try/except
        self.client.write_registers(self.input_register, message, unit=self.slave_id)
    
    # implemented provisional fix from https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/240#issuecomment-699752688
    def request_status(self, num_bytes=6, rtlimit=50):
        retries = -1
        status = None
        num_regs = int(ceil(num_bytes/2.0))
        
        # Get status from the device (too aggressive?)
        while not isinstance(status, ReadHoldingRegistersResponse) and retries < rtlimit and not rospy.is_shutdown():
            rtlimit += 1
            try:
                if retries >= rtlimit:
                    print(f'Could not get device status after {rtlimit} retries. No reponse received.')
                    return None
                status = self.client.read_holding_registers(self.output_register, num_regs, unit=self.slave_id, timeout=3)   
                rospy.sleep(0.05)             
            except:
                print(f'Known register reading error occured. Retrying... ({rtlimit - retries} attempt(s) left)')
        
        output = []
        
        for i in range(num_regs):
            output.append((status.getRegister(i) & 0xFF00) >> 8)
            output.append( status.getRegister(i) & 0x00FF)           
        
        return output

class RobotiqTCPClient:
    def __init__(self):
        raise NotImplementedError