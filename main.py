# Main Program 
# +++++++++++++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++++++++++++

from can_bus import CAN_setting
from bridge_plc_modbus import bridge_hub
from pid import PID
import json

import threading
import math
import time

class main_robot():
   
    error = 0
    mode_control = True
    sensor_posisi = [False, False]
    Kecepatan_base = 0.0
    Kecepatan_manual = 0.0

    def __init__(self):
        self.canbus = CAN_setting()
        self.bridge = bridge_hub()
        self.pid = PID(self.bridge.PID[0], self.bridge.PID[1], self.bridge.PID[2])
    
        threading.Thread(target=self.data_read).start()
        threading.Thread(target=self.data_plc).start()

        self.previous_time = time.time() * 1000
        self.turn = False

    def robot(self):
        # print(str(self.canbus.flag[-2]) + " " + str(self.canbus.flag[-3]) + " " + str(self.canbus.data_RFID))
        if self.canbus.data_voltage[0] > self.bridge.low_voltage and self.canbus.data_voltage[1] > self.bridge.low_voltage:

            # Mode Kontrol Robot
            if self.bridge.manual_b == True:
                self.mode_control = True
            elif self.bridge.manual_b == False:
                self.mode_control = False

            # Robot Main
            if self.mode_control == False and self.bridge.start_b == True:     
                if self.canbus.flag[-2] != bin(0)[2:] or self.turn == True:
                    if self.bridge.Obstacel_data[1] == False:
                        if self.bridge.Obstacel_data[0] == True:
                            self.bridge.buzzer_select('S1')
                            self.Kecepatan_base = self.convert_to_RPM(self.bridge.Kecepatan * 1/5)
                        else:
                            self.bridge.buzzer_select('S4')
                            self.mode_run('/home/runindo/riset/AGV_Robot/rule/node.json', self.canbus.data_RFID)                        
                    else:
                        self.Kecepatan_base = 0.0
                        self.bridge.buzzer_select('S2')
                        self.canbus.set_kecepatan_motor([int(self.Kecepatan_base - self.pid.compute(self.error)), int(-self.Kecepatan_base - self.pid.compute(self.error))])
                    if self.turn == False:
                        self.canbus.set_kecepatan_motor([int(self.Kecepatan_base - self.pid.compute(self.error)), int(-self.Kecepatan_base - self.pid.compute(self.error))])
                else:
                    self.Kecepatan_base = 0.0
                    self.bridge.buzzer_select('S2')
                    self.canbus.set_kecepatan_motor([int(self.Kecepatan_base - self.pid.compute(self.error)), int(-self.Kecepatan_base - self.pid.compute(self.error))])
            else:
                if self.bridge.Kontrol_data[0] == True:
                    self.canbus.set_kecepatan_motor([int(self.convert_to_RPM(self.bridge.c_kecepatan)), -int(self.convert_to_RPM(self.bridge.c_kecepatan))])
                elif self.bridge.Kontrol_data[3] == True:
                    self.canbus.set_kecepatan_motor([int(self.convert_to_RPM(self.bridge.c_kecepatan)), int(self.convert_to_RPM(self.bridge.c_kecepatan))])
                elif self.bridge.Kontrol_data[2] == True: 
                    self.canbus.set_kecepatan_motor([-int(self.convert_to_RPM(self.bridge.c_kecepatan)), -int(self.convert_to_RPM(self.bridge.c_kecepatan))])
                elif self.bridge.Kontrol_data[1] == True:
                    self.canbus.set_kecepatan_motor([-int(self.convert_to_RPM(self.bridge.c_kecepatan)), int(self.convert_to_RPM(self.bridge.c_kecepatan))])
                else:
                    self.Kecepatan_base = 0.0
                    self.bridge.buzzer_select('None')
                    self.canbus.set_kecepatan_motor([int(self.Kecepatan_base - self.pid.compute(self.error)), int(-self.Kecepatan_base - self.pid.compute(self.error))])
        else:
            self.Kecepatan_base = 0.0
            self.bridge.buzzer_select('S2')
            self.canbus.set_kecepatan_motor([int(self.Kecepatan_base), int(-self.Kecepatan_base)])
            # self.canbus.set_kecepatan_motor([int(self.Kecepatan_base - self.pid.compute(self.error)), int(-self.Kecepatan_base - self.pid.compute(self.error))])
        
        self.bridge.rule_setting()
        self.bridge.set_ID_card()
        self.select_track(self.sensor_posisi)
        if self.bridge.b_calibration_s == True:
            self.canbus.calibrate_sensor_magnet()

    def data_read(self):
        while True:
            self.canbus.read_data_sensor()

    def data_plc(self):
        while True:
            self.pid = PID(self.bridge.PID[0], self.bridge.PID[1], self.bridge.PID[2])
            self.bridge.parameter_data()
            self.bridge.tegangan_robot(self.canbus.data_voltage[1], self.canbus.data_current[1], self.canbus.data_RFID)
            self.bridge.tegangan_robot(self.canbus.data_voltage[0], self.canbus.data_current[0], self.canbus.data_RFID)
            self.bridge.sensor_magnet_data(self.canbus.sensor[0], self.canbus.sensor[1])
    
    def convert_to_RPM(self, data):
        return data * 60 * 25 / self.bridge.wheel_diameter / math.pi
    
    def action_mode(self, action):
        if action == "Stop":
            self.Kecepatan_base = 0.0
        elif action == "LeftL":
            self.Kecepatan_base = self.convert_to_RPM(0.10)
            if self.canbus.flag[-3] == bin(1)[2:]:
                self.turn = True
                self.previous_time = time.time() * 1000
            if (time.time() * 1000 - self.previous_time) < 2000:
                self.canbus.set_kecepatan_motor([-int(self.Kecepatan_base), -int(self.Kecepatan_base)])
            else:
                if self.canbus.flag[-2] == bin(0)[2:]:
                    self.canbus.set_kecepatan_motor([-int(self.Kecepatan_base * 2/5), -int(self.Kecepatan_base * 2/5)])
                else:
                    self.turn = False
        elif action == "RightL":
            self.Kecepatan_base = self.convert_to_RPM(0.10)
            if self.canbus.flag[-4] == bin(1)[2:]:
                self.turn = True
                self.previous_time = time.time() * 1000
            if (time.time() * 1000 - self.previous_time) < 2000:
                self.canbus.set_kecepatan_motor([int(self.Kecepatan_base), int(self.Kecepatan_base)])
            else:
                if self.canbus.flag[-2] == bin(0)[2:]:
                    self.canbus.set_kecepatan_motor([int(self.Kecepatan_base * 2/5), int(self.Kecepatan_base * 2/5)])
                else:
                    self.turn = False
        elif action == "LeftR":
            self.sensor_posisi = [0, 1]
            self.Kecepatan_base = self.convert_to_RPM(0.10)
        elif action == "RightR":
            self.sensor_posisi = [1, 0]
            self.Kecepatan_base = self.convert_to_RPM(0.10)

    def mode_run(self, file_path, key):
        with open(file_path, 'r') as file:
            config = json.load(file)
        if str(key) in config:
            self.action_mode(config[str(key)])
        else: 
            self.turn = False
            self.sensor_posisi = [1, 0]
            self.Kecepatan_base = self.convert_to_RPM(self.bridge.Kecepatan)
    
    def select_track(self, track):
        if track[0] == 1 and track[1] == 0:
            self.error = self.canbus.right_data_sensor()
            self.bridge.direction_indicator('right')
        elif track[0] == 0 and track[1] == 1:
            self.error = self.canbus.left__data_sensor()
            self.bridge.direction_indicator('left')
        else:
             self.error = self.canbus.center_data_sensor()
             self.bridge.direction_indicator('center')

if __name__ == '__main__':
    app = main_robot()
    try:
        while True:
            app.robot()
    except KeyboardInterrupt:
        print('Stopped by keyboard interrupt')
        app.bridge.client.close()
        app.bridge.RFID_ss.ser.close()
        pass
    except BaseException:
        app.bridge.client.close()
        app.bridge.RFID_ss.ser.close()
        print('Stopped by exception')
        raise
        