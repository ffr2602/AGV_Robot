from pymodbus.client.serial import ModbusSerialClient
import struct
import serial

m_reg = 3072
v_reg = 512

class bridge_hub():

    PID = [0.0, 0.0, 0.0]
    Kecepatan = 0.0
    c_kecepatan = 0.0
    low_voltage = 0.0
    wheel_diameter = 0.0
    dir_data = [False, False, False]
    Kontrol_data = [False, False, False, False]
    Obstacel_data = [False, False]

    NODE_data = [False, False, False]

    start_b = False
    manual_b = False
    set_ID_b = False
    b_calibration_s = False
    b_unlock = False

    def __init__(self):
        self.client = ModbusSerialClient(port='/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0', baudrate=115200, parity='N', stopbits=1)
        self.client.connect()
        self.RFID_ss = RFID_setting()
        self.connection = self.client.connected 

    def parameter_data(self):
        self.PID[0] = self.registers_to_float(self.client.read_holding_registers((v_reg + 1000), 2, 1).registers)
        self.PID[1] = self.registers_to_float(self.client.read_holding_registers((v_reg + 1002), 2, 1).registers)
        self.PID[2] = self.registers_to_float(self.client.read_holding_registers((v_reg + 1004), 2, 1).registers)
        self.Kecepatan = self.registers_to_float(self.client.read_holding_registers((v_reg + 1006), 2, 1).registers)
        self.wheel_diameter = self.registers_to_float(self.client.read_holding_registers((v_reg + 1008), 2, 1).registers)
        self.low_voltage = self.registers_to_float(self.client.read_holding_registers((v_reg + 1010), 2, 1).registers)
        self.c_kecepatan = self.registers_to_float(self.client.read_holding_registers((v_reg + 1012), 2, 1).registers)

        self.Obstacel_data = self.client.read_coils((m_reg + 11), 2, 1).bits
        self.Kontrol_data = self.client.read_coils((m_reg + 22), 4, 1).bits
        self.manual_b = self.client.read_coils((m_reg + 76), 1, 1).bits[0]
        self.set_ID_b = self.client.read_coils((m_reg + 78), 1, 1).bits[0]
        self.start_b = self.client.read_coils(m_reg, 1, 1).bits[0]
        self.NODE_data = self.client.read_coils((m_reg + 80), 3, 1).bits
        self.b_calibration_s = self.client.read_coils((m_reg + 84), 1, 1).bits[0]
        self.b_unlock = self.client.read_coils((m_reg + 85), 1, 1).bits[0]

    def tegangan_robot(self, tegangan, arus, ac_ID):
        if self.connection:
            self.client.write_registers((v_reg + 8), self.float_to_registers(tegangan), 1)
            self.client.write_registers((v_reg + 10), self.float_to_registers(arus), 1) 
            self.client.write_register((v_reg + 14), ac_ID, 1)   

    def rule_setting(self):
        if self.connection:
            if self.NODE_data[2] == True:
                for i in range(5):
                    self.client.write_register((v_reg + 16 + i), 0, 1)
                
    def buzzer_select(self, data):
        if self.connection:
            if data == 'S1':
                voice = [1, 0, 0, 0]
            elif data == 'S2':
                voice = [0, 1, 0, 0]
            elif data == 'S3':
                voice = [0, 0, 1, 0]
            elif data == 'S4':
                voice = [0, 0, 0, 1]
            else:
                voice = [0, 0, 0, 0]
            self.client.write_coils((m_reg + 3), voice, 1)
    
    def convert_signed(self, num):
        if num < 0:
            return 0xFFFF + num + 1
        else:
            return num
    
    def sensor_magnet_data(self, right, left):
        if self.connection:
            self.client.write_register((v_reg + 12), self.convert_signed(right), 1)
            self.client.write_register((v_reg + 13), self.convert_signed(left), 1)
    
    def direction_indicator(self, data):
        if data == 'right':
            dir_data = [1, 0]
        elif data == 'left':
            dir_data = [0, 1]
        else:
            dir_data = [0, 0]
        self.client.write_coils((m_reg + 27), dir_data, 1)
    
    def set_ID_card(self):
        if self.connection:
            if self.set_ID_b == True:
                self.RFID_ss.set_ID(int(self.client.read_holding_registers((v_reg + 15), 1, 1).registers[0]))

    def float_to_registers(self, float_value):
        float_bytes = struct.pack('>f', float_value)  
        reg1, reg2 = struct.unpack('>HH', float_bytes)  
        return [reg2, reg1]
    
    def registers_to_float(self, registers):
        reg1, reg2 = registers
        float_bytes = struct.pack('>HH', reg2, reg1) 
        float_value = struct.unpack('>f', float_bytes)[0]
        return float_value
   
class RFID_setting():
    def __init__(self) -> None:
        self.ser = None
        self.ser = serial.Serial()
        self.ser.port = '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0'
        self.ser.baudrate = 9600
        self.ser.timeout = 1000
        self.ser.open()

        if self.ser.is_open:
           bufSend = [170, 85, 4, 0, 0, 1, 171]
           self.ser.write(bufSend)

    def set_ID(self, ID):
        if self.ser.is_open:
            SetID = int(ID)
            parset = [1, 2, 3, 4, 5, 6, 7, 8, 172]
            parset[4] = SetID >> 56 & 255
            parset[5] = SetID >> 48 & 255
            parset[6] = SetID >> 40 & 255
            parset[7] = SetID >> 32 & 255
            parset[0] = SetID >> 24 & 255
            parset[1] = SetID >> 16 & 255
            parset[2] = SetID >> 8 & 255
            parset[3] = SetID >> 0 & 255
            bufSend = [170, 81, 11, 0, 0]
            bufSend.extend(parset)
            self.ser.write(bufSend)
    

