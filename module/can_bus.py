import can


class CAN_setting():
    def __init__(self):
        self.can_open : bool = False
        self.sensor : list[int] = [0, 0, 0]
        self.voltage : float = 24.00
        self.temp_driver : float = 0.0
        self.data_RFID : int = 0
        self.strengt_RFID : int = 0
        self.flag : int = 0
        self.error : list[int] = [0, 0]
        self.current : list[int] = [0, 0]


    def set_kecepatan_motor(self, speed:list[int, int]):
        if self.can_open == True:
            self.bus.send(can.Message(arbitration_id=0x201, data=[0x0f, 0x00, 0x0f, 0x00, 0x03], is_extended_id=False))
            self.bus.send(can.Message(arbitration_id=0x301, data=[int(hex(speed[0] & 0xff), 16),
                                                                int(hex(speed[0] >> 8 & 0xff), 16),
                                                                int(hex(speed[0] >> 16 & 0xff), 16),
                                                                int(hex(speed[0] >> 32 & 0xff), 16),
                                                                int(hex(speed[1] & 0xff), 16),
                                                                int(hex(speed[1] >> 8 & 0xff), 16),
                                                                int(hex(speed[1] >> 16 & 0xff), 16),
                                                                int(hex(speed[1] >> 32 & 0xff), 16)], is_extended_id=False))
                            
                
    def read_data_sensor(self):
        try:
            self.bus = can.Bus(interface='socketcan', channel='can0', bitrate=500000)
            msg_recv = self.bus.recv(timeout=1.0)
            if msg_recv is None:
                self.can_open = False
            else:
                self.can_open = True
                if msg_recv.arbitration_id == 0x181:
                    self.error = [msg_recv.data[0] << 0 | msg_recv.data[1] << 8, msg_recv.data[2] << 0 | msg_recv.data[3] << 8]
                    self.current = [msg_recv.data[4] << 0 | msg_recv.data[5] << 8, msg_recv.data[6] << 0 | msg_recv.data[7] << 8]
                if msg_recv.arbitration_id == 0x281:
                    self.voltage = (msg_recv.data[0] << 0 | msg_recv.data[1] << 8 | msg_recv.data[2] << 16 | msg_recv.data[3] << 32) * 0.001
                    self.temp_driver = (msg_recv.data[4] << 0 | msg_recv.data[5] << 8)
                if msg_recv.arbitration_id == 0x186:
                    right_track = msg_recv.data[2] - msg_recv.data[3]
                    left__track = msg_recv.data[0] - msg_recv.data[1]
                    self.sensor = [left__track, int((right_track + left__track) * 0.5), right_track]
                    self.flag = msg_recv.data[4]
                if msg_recv.arbitration_id == 0x009:
                    if msg_recv.data[0] == 1:
                        self.data_RFID = msg_recv.data[4] << 24 |  msg_recv.data[5] << 16 |  msg_recv.data[6] << 8 |  msg_recv.data[7] << 0
                        self.strengt_RFID = msg_recv.data[3]
        except Exception as e:
            print("Error CAN BUS: ", e)
            self.can_open = False
            
            
    def calibrate_sensor_magnet(self):
        if self.can_open == True:
            self.bus.send(can.Message(arbitration_id=0x606, data=[0x2f, 0x20, 0x20, 0x00], is_extended_id=False))
    