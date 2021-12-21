#!/usr/bin/evn python3

from pymodbus.exceptions import ModbusIOException
import rospy

from pymodbus.client.sync import ModbusSerialClient, ModbusTcpClient
from pymodbus.register_read_message import ReadHoldingRegistersResponse
from math import ceil

class RobotiqRTUClient:
    def __init__(self, unit_id = 0x0009, input_addr = 0x03E8, output_addr = 0x07D0):
        self.client = None
        self.unit_id = unit_id
        self.input_register_addr = input_addr
        self.output_register_addr = output_addr
    
    def connect(self, device_addr, delay = 1):
        self.client = ModbusSerialClient(method = 'rtu',
                                         port = device_addr,
                                         stopbits = 1,
                                         bytesize = 8,
                                         baudrate = 115200,
                                         timeout = 0.2)
        self.client.connect() # this is always true?
    
    def disconnect(self):
        if self.client != None:
            self.client.close()
    
    def send_command(self, command):
        # make sure data has an even number of elements
        if(len(command) % 2 == 1):
            command.append(0)
        
        # empty output register value to write
        output = []
        
        # fill message by combining two bytes in one register
        for i in range(0, len(command)//2):
            byte1 = command[2 * i] << 8
            byte2 = command[2 * i + 1]
            output.append(byte1 + byte2)
            
        # sending the command
        # TODO: catch ModbusIOException when writing powered off
        self.client.write_registers(self.input_register_addr, output, unit=self.unit_id)

    
    def request_status(self, nbytes = 6, retries = 50):
        nregs = int(ceil(nbytes // 2.0))
        # TODO: catch ModbusIOException when reading powered off
        status = self.client.read_holding_registers(self.output_register_addr, nregs, unit = self.unit_id, timeout = 3)
        return self.parse_register(status, nregs)
        
    def parse_register(self, recv_regs, nregs):
        output = []
        for i in range(nregs):
            output.append((recv_regs.getRegister(i) & 0xFF00) >> 8)
            output.append( recv_regs.getRegister(i) & 0x00FF)           
        return output
    
class RobotiqTCPClient:
    def __init__(self):
        raise NotImplementedError