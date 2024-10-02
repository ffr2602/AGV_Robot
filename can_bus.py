import can 
import numpy as np
import time 

CAN_ID_first_set = [0x201, 0x202]
CAN_ID = [0x501, 0x502]
CAN_ID_sensor = 0x186
CAN_ID_power = [0x481, 0x482]
CAN_ID_RFID = 0x009

class CAN_setting():
    def __init__(self):
        self.can_open = False
        try:
            self.bus = can.Bus(interface='socketcan', channel='can0', bitrate=500000)
            self.can_open = True
        except OSError:
            self.can_open = False
            exit()
        if self.can_open == True:
            for i in range(len(CAN_ID_first_set)):
                self.bus.send(can.Message(arbitration_id=0x000, data=[0x01, i], extended_id=False))
                self.bus.send(can.Message(arbitration_id=CAN_ID_first_set[i], data=[0x0f,0x00,0x00,0x00,0x01,0x00], extended_id=False))
                time.sleep(0.5)
            for i in range(len(CAN_ID_first_set)):
                self.bus.send(can.Message(arbitration_id=CAN_ID_first_set[i], data=[0x0f,0x00,0x00,0x00,0x00,0x00], extended_id=False))
                time.sleep(0.5)

        # Variabel Data Sensor
        self.sensor = np.zeros(3).astype(int)
        self.data_voltage = np.zeros(2).astype(float)
        self.data_current = np.zeros(2).astype(float)
        self.data_RFID = 0
        self.strengt_RFID = 0
        self.flag = bin(0)

    def set_kecepatan_motor(self, speed):
        if self.can_open == True:
            for i in range(len(speed)):
                self.bus.send(can.Message(arbitration_id=CAN_ID[i], data=[0x0f,0x00,
                                                                          int(hex(speed[i] & 0xff), 16),
                                                                          int(hex(speed[i] >> 8 & 0xff), 16),
                                                                          int(hex(speed[i] >> 16 & 0xff), 16),
                                                                          int(hex(speed[i] >> 32 & 0xff), 16),0x00,0x00], extended_id=False))
            time.sleep(0.005)
                
    def read_data_sensor(self):
        if self.can_open == True:
            msg_recv = self.bus.recv()
            if msg_recv.arbitration_id == CAN_ID_sensor:
                right_track = msg_recv.data[0] - msg_recv.data[1]
                left__track = msg_recv.data[2] - msg_recv.data[3]
                self.sensor = [int(right_track), int(left__track), int((right_track + left__track) * 0.5)]
                self.flag = bin(msg_recv.data[4])[2:]
            if msg_recv.arbitration_id == CAN_ID_power[0]:
                self.data_voltage[0] = (msg_recv.data[0] << 0 | msg_recv.data[1] << 8) * 0.1
                self.data_current[0] = (msg_recv.data[2] << 0 | msg_recv.data[3] << 8 | msg_recv.data[4] << 16) * 0.001
            if msg_recv.arbitration_id == CAN_ID_power[1]:
                self.data_voltage[1] = (msg_recv.data[0] << 0 | msg_recv.data[1] << 8) * 0.1
                self.data_current[1] = (msg_recv.data[2] << 0 | msg_recv.data[3] << 8 | msg_recv.data[4] << 16) * 0.001
            if msg_recv.arbitration_id == CAN_ID_RFID:
                if msg_recv.data[0] == 1:
                    self.data_RFID = msg_recv.data[4] << 24 |  msg_recv.data[5] << 16 |  msg_recv.data[6] << 8 |  msg_recv.data[7] << 0
                    self.strengt_RFID = msg_recv.data[3]
            
    def center_data_sensor(self):
        return self.sensor[2]
    
    def left__data_sensor(self):
        return self.sensor[0]
    
    def right_data_sensor(self):
        return self.sensor[1]
    
    def calibrate_sensor_magnet(self):
        if self.can_open == True:
            self.bus.send(can.Message(arbitration_id=0x606, data=[0x2f, 0x20, 0x20, 0x00], extended_id=False))
    
    def break_system(self):
        if self.can_open == True:
            self.bus.send(can.Message(arbitration_id=0x000, data=[0x80, 0x00], extended_id=False))
    
    
        





