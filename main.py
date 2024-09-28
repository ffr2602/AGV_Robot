# Main Program 
# +++++++++++++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++++++++++++

from can_bus import CAN_setting
from bridge_plc_modbus import bridge_hub
from pid import PID

import threading
import numpy as np
import math
import time

class main_robot():
   
    error = 0
    mode_control = True
    sensor_posisi = [False, False, False]
    kecepatan_motor = np.zeros(2).astype(int)
    referensi_voltage = 22.50
    diameter_roda = 0.11
    kecepatan_low = 0.0

    Kecepatan_base = 0.0

    def __init__(self):
        self.canbus = CAN_setting()
        self.bridge = bridge_hub()

        self.pid = PID(kp=self.bridge.PID[0], kd=self.bridge.PID[2], ki=self.bridge.PID[1])
    
        threading.Thread(target=self.data_read).start()
        threading.Thread(target=self.data_plc).start()

        self.sensor_posisi[1] = True

    def robot(self):
        self.pid = PID(kp=self.bridge.PID[0], kd=self.bridge.PID[2], ki=self.bridge.PID[1])
        if self.canbus.data_voltage[0] > self.referensi_voltage and self.canbus.data_voltage[1] > self.referensi_voltage:
            # Mode Kontrol Robot
            if self.bridge.manual_b == True:
                self.mode_control = True
            elif self.bridge.manual_b == False:
                self.mode_control = False

            # Robot Main
            if self.mode_control == False and self.bridge.start_b == True:            
                if self.canbus.flag__data_sensor() != 16:
                    if self.bridge.Obstacel_data[1] == False:
                        if self.bridge.Obstacel_data[0] == True:
                            self.bridge.buzzer_select('S1')
                            if self.kecepatan_motor[0] != 0 and self.kecepatan_motor[1] != 0:
                                self.kecepatan_low = self.convert_to_RPM(self.bridge.Kecepatan) * 2/4
                        else:
                            self.kecepatan_low = 0.0
                            self.bridge.buzzer_select('S4')
                        
                        if self.canbus.data_RFID == 1:
                            self.kecepatan_low = 0.0
                            self.Kecepatan_base = 0.0
                            self.kecepatan_motor[0] = int(self.Kecepatan_base - self.pid.compute(self.error))
                            self.kecepatan_motor[1] = int(-self.Kecepatan_base - self.pid.compute(self.error))
                            self.canbus.set_kecepatan_motor(self.kecepatan_motor)
                        else:
                            if self.canbus.data_RFID == 2:
                                data = (self.convert_to_RPM(self.bridge.Kecepatan) * 4/5)
                                self.kecepatan_motor[0] = int(self.convert_to_RPM(self.bridge.Kecepatan) - self.pid.compute(self.error) - data)
                                self.kecepatan_motor[1] = int(-self.convert_to_RPM(self.bridge.Kecepatan) - self.pid.compute(self.error) + data)
                                self.canbus.set_kecepatan_motor(self.kecepatan_motor)
                            else:
                                self.kecepatan_motor[0] = int(self.convert_to_RPM(self.bridge.Kecepatan) - self.pid.compute(self.error) - self.kecepatan_low)
                                self.kecepatan_motor[1] = int(-self.convert_to_RPM(self.bridge.Kecepatan) - self.pid.compute(self.error) + self.kecepatan_low)
                                self.canbus.set_kecepatan_motor(self.kecepatan_motor)
                    else:
                        self.kecepatan_low = 0.0
                        self.Kecepatan_base = 0.0
                        self.kecepatan_motor[0] = int(self.Kecepatan_base - self.pid.compute(self.error))
                        self.kecepatan_motor[1] = int(-self.Kecepatan_base - self.pid.compute(self.error))
                        self.canbus.set_kecepatan_motor(self.kecepatan_motor)
                        self.bridge.buzzer_select('S2')
                else:
                    self.kecepatan_low = 0.0
                    self.Kecepatan_base = 0.0
                    self.kecepatan_motor[0] = int(self.Kecepatan_base - self.pid.compute(self.error))
                    self.kecepatan_motor[1] = int(-self.Kecepatan_base - self.pid.compute(self.error))
                    self.canbus.set_kecepatan_motor(self.kecepatan_motor)
                    self.bridge.buzzer_select('S2')
            else:
                if self.bridge.Kontrol_data[0] == True:
                    self.kecepatan_motor = [int(self.convert_to_RPM(self.bridge.c_kecepatan)), -int(self.convert_to_RPM(self.bridge.c_kecepatan))]
                    self.canbus.set_kecepatan_motor(self.kecepatan_motor)
                elif self.bridge.Kontrol_data[3] == True:
                    self.kecepatan_motor = [int(self.convert_to_RPM(self.bridge.c_kecepatan)), int(self.convert_to_RPM(self.bridge.c_kecepatan))]
                    self.canbus.set_kecepatan_motor(self.kecepatan_motor)
                elif self.bridge.Kontrol_data[2] == True:
                    self.kecepatan_motor = [-int(self.convert_to_RPM(self.bridge.c_kecepatan)), -int(self.convert_to_RPM(self.bridge.c_kecepatan))]
                    self.canbus.set_kecepatan_motor(self.kecepatan_motor)
                elif self.bridge.Kontrol_data[1] == True:
                    self.kecepatan_motor = [-int(self.convert_to_RPM(self.bridge.c_kecepatan)), int(self.convert_to_RPM(self.bridge.c_kecepatan))]
                    self.canbus.set_kecepatan_motor(self.kecepatan_motor)
                else:
                    self.kecepatan_low = 0.0
                    self.Kecepatan_base = 0.0
                    self.kecepatan_motor[0] = int(self.Kecepatan_base - self.pid.compute(self.error))
                    self.kecepatan_motor[1] = int(-self.Kecepatan_base - self.pid.compute(self.error))
                    self.canbus.set_kecepatan_motor(self.kecepatan_motor)
                    self.bridge.buzzer_select('None')
        else:
            self.Kecepatan_base = 0.0
            self.kecepatan_motor[0] = int(self.Kecepatan_base - self.pid.compute(self.error))
            self.kecepatan_motor[1] = int(-self.Kecepatan_base - self.pid.compute(self.error))
            self.canbus.set_kecepatan_motor(self.kecepatan_motor)
            self.bridge.buzzer_select('S2')
        
        if self.sensor_posisi[0] == True:
            self.error = self.canbus.right_data_sensor()
            self.sensor_posisi[1] = False
            self.sensor_posisi[2] = False

        elif self.sensor_posisi[1] == True:
            self.error = self.canbus.left__data_sensor()
            self.sensor_posisi[0] = False
            self.sensor_posisi[2] = False

        elif self.sensor_posisi[2] == True:
            self.error = self.canbus.center_data_sensor()
            self.sensor_posisi[0] = False
            self.sensor_posisi[1] = False
        
        if self.bridge.dir_data[0] == True:
            self.sensor_posisi[0] = True

        elif self.bridge.dir_data[1] == True:
            self.sensor_posisi[1] = True

        elif self.bridge.dir_data[2] == True:
            self.sensor_posisi[2] = True
        
        if self.bridge.set_ID_b == True:
            self.bridge.set_ID_card()
        
    def data_read(self):
        while True:
            self.canbus.read_data_sensor()

    def data_plc(self):
        while True:
            self.bridge.parameter_data()
            self.bridge.tegangan_robot(self.canbus.data_voltage[1], self.canbus.data_current[1], self.canbus.data_RFID)
            self.bridge.tegangan_robot(self.canbus.data_voltage[0], self.canbus.data_current[0], self.canbus.data_RFID)
    
    def convert_to_RPM(self, data):
        return data * 60 * 25 / self.diameter_roda / math.pi
            

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
        print('Stopped by exception')
        raise
        