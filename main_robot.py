import asyncio
from asyncio import threads
from module.database import *
from module.konversi import *
from module.can_bus import CAN_setting
from module.pid import PID
import json
import time
import struct

class robot():

    error = 0
    temp_buzzer = 0
    buzzer = False
    akselerasi = False
    temp_RFID = 0
    temp_SPEED = 0

    current_speed = [0, 0] 
    acceleration_rate = 35

    wheel_temp = False

    output : list[bool] = [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False]

    previous_time = 0

    def __init__(self):
        with open(setting_path, 'r') as file:
            config = json.load(file)
            set_value(1, 25, [config['Obstacle']])
            set_value(3, 40, [config['Speed']])
            set_value(3, 222, [config['ID_CAR']])
            set_value(3, 1000, [config['Acceleration']])
            set_value(3, 1001, [config['Default_Music']])
            set_value(3, 1010, [config['Max_Speed']])
            set_value(3, 1011, [config['Select_Track']])
            set_value(3, 1012, float32_to_registers(config['Low_Voltage']))
        set_value(1, 2, [1])
        set_value(2, 0, [1])
        self.temp_buzzer = config['Default_Music']
        self.interval = config['Acceleration']
        self.track_select = config['Select_Track']
        self.pid = PID(config['P'], config['I'], config['D'])
        self.pid_slow = PID(9.00, 0.00, 0.00)
        self.canbus = CAN_setting()
    
    async def run_robot(self):  
        while True:
            if self.canbus.data_RFID != self.temp_RFID:
                self.temp_RFID = self.canbus.data_RFID
                with open(file_path, 'r') as node:
                    data_node = json.load(node)
                if str(get_value(3, 38, 1)[0]) in data_node:
                    if str(self.canbus.data_RFID) in data_node[str(get_value(3, 38, 1)[0])]:
                        for i in range(len(data_node[str(get_value(3, 38, 1)[0])][str(self.canbus.data_RFID)])):
                            set_value(2, 1, [1])
                            if data_node[str(get_value(3, 38, 1)[0])][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'forward':
                                self.interval = data_node[str(get_value(3, 38, 1)[0])][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]
                                set_value(1, 1, [1])

                            elif data_node[str(get_value(3, 38, 1)[0])][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'stop':
                                self.interval = data_node[str(get_value(3, 38, 1)[0])][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]
                                set_value(1, 0, [1])

                            elif data_node[str(get_value(3, 38, 1)[0])][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'music':
                                self.temp_buzzer = data_node[str(get_value(3, 38, 1)[0])][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]

                            elif data_node[str(get_value(3, 38, 1)[0])][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'stick':
                                set_value(1, 19, [data_node[str(get_value(3, 38, 1)[0])][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]])
                            
                            elif data_node[str(get_value(3, 38, 1)[0])][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'speed':
                                set_value(3, 40, [data_node[str(get_value(3, 38, 1)[0])][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]])
                                self.akselerasi = False

                            elif data_node[str(get_value(3, 38, 1)[0])][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'obs set':
                                set_value(1, 25, [data_node[str(get_value(3, 38, 1)[0])][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]])
                            
                            elif data_node[str(get_value(3, 38, 1)[0])][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'route trans':
                                set_value(3, 38, [data_node[str(get_value(3, 38, 1)[0])][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]])
                                break
                            
                            elif data_node[str(get_value(3, 38, 1)[0])][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'turn':
                                self.track_select = data_node[str(get_value(3, 38, 1)[0])][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]
                            
                            elif data_node[str(get_value(3, 38, 1)[0])][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'delay':
                                await asyncio.sleep(data_node[str(get_value(3, 38, 1)[0])][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1])
                        set_value(2, 1, [0])
            await asyncio.sleep(0.001)

    async def main_robot(self):
        while True:
            self.ethercat.send_processdata()
            input_bits = struct.unpack('<H', self.el1809.input)[0]
            wkc = self.ethercat.receive_processdata(1000)
            if wkc < 0:
                continue
            if (input_bits >> 2) & 0x01 != self.wheel_temp:
                self.wheel_temp = (input_bits >> 2) & 0x01
                if (input_bits >> 2) & 0x01 == False:
                    if get_value(1, 11, 1)[0] == 0:
                        set_value(1, 11, [1])
                    else:
                        set_value(1, 11, [0])

            self.canbus.read_data_sensor()
            set_value(4, 12, get_uptime())
            set_value(4, 0, float32_to_registers(self.canbus.voltage))
            set_value(4, 2, float32_to_registers(get_cpu_temperature()))
            set_value(4, 4, self.canbus.error)
            set_value(4, 6, float32_to_registers(struct.unpack("h", struct.pack("H", self.canbus.current[1]))[0]))
            set_value(4, 8, float32_to_registers(struct.unpack("h", struct.pack("H", self.canbus.current[0]))[0]))
            set_value(4, 10, float32_to_registers(self.canbus.temp_driver))
            set_value(2, 4, [self.canbus.can_open])
            set_value(3, 39, [self.canbus.data_RFID])
            set_value(1, 2, [(input_bits >> 3) & 0x01]) 
            set_value(2, 2, [not (self.canbus.flag >> 1) & 1])
            set_value(2, 9, [(self.canbus.flag >> 7) & 1])
            set_value(4, 23, [self.track_select])

            set_value(1, 7, [(input_bits >> 6) & 0x01, (input_bits >> 7) & 0x01, (input_bits >> 8) & 0x01, (input_bits >> 9) & 0x01]) 

            if get_value(1, 11, 1)[0]:
                if (input_bits >> 9) & 0x01:
                    self.output[8] = 1
                    self.output[6] = 1
                    output_bits = self.output[0] << 0 | self.output[1] << 1 | self.output[2] << 2 | self.output[3] << 3 | self.output[4] << 4 | self.output[5] << 5 | self.output[6] << 6 | self.output[7] << 7 | self.output[8] << 8 | self.output[9] << 9 | self.output[10] << 10 | self.output[11] << 11 | self.output[12] << 12 | self.output[13] << 13 | self.output[14] << 14 | self.output[15] << 15
                    self.el2809.output = struct.pack('<H', output_bits)
                    self.ethercat.send_processdata()
                elif (input_bits >> 8) & 0x01 == 0 and (input_bits >> 9) & 0x01 == 0:
                    self.output[8] = 1
                    self.output[6] = 1
                    output_bits = self.output[0] << 0 | self.output[1] << 1 | self.output[2] << 2 | self.output[3] << 3 | self.output[4] << 4 | self.output[5] << 5 | self.output[6] << 6 | self.output[7] << 7 | self.output[8] << 8 | self.output[9] << 9 | self.output[10] << 10 | self.output[11] << 11 | self.output[12] << 12 | self.output[13] << 13 | self.output[14] << 14 | self.output[15] << 15
                    self.el2809.output = struct.pack('<H', output_bits)
                    self.ethercat.send_processdata()
                elif (input_bits >> 8) & 0x01:
                    self.output[8] = 0
                    self.output[6] = 0
                    output_bits = self.output[0] << 0 | self.output[1] << 1 | self.output[2] << 2 | self.output[3] << 3 | self.output[4] << 4 | self.output[5] << 5 | self.output[6] << 6 | self.output[7] << 7 | self.output[8] << 8 | self.output[9] << 9 | self.output[10] << 10 | self.output[11] << 11 | self.output[12] << 12 | self.output[13] << 13 | self.output[14] << 14 | self.output[15] << 15
                    self.el2809.output = struct.pack('<H', output_bits)
                    self.ethercat.send_processdata()
            else:
                if (input_bits >> 8) & 0x01:
                    self.output[8] = 1
                    self.output[6] = 1
                    output_bits = self.output[0] << 0 | self.output[1] << 1 | self.output[2] << 2 | self.output[3] << 3 | self.output[4] << 4 | self.output[5] << 5 | self.output[6] << 6 | self.output[7] << 7 | self.output[8] << 8 | self.output[9] << 9 | self.output[10] << 10 | self.output[11] << 11 | self.output[12] << 12 | self.output[13] << 13 | self.output[14] << 14 | self.output[15] << 15
                    self.el2809.output = struct.pack('<H', output_bits)
                    self.ethercat.send_processdata()
                elif (input_bits >> 8) & 0x01 == 0 and (input_bits >> 9) & 0x01 == 0:
                    self.output[8] = 1
                    self.output[6] = 1
                    output_bits = self.output[0] << 0 | self.output[1] << 1 | self.output[2] << 2 | self.output[3] << 3 | self.output[4] << 4 | self.output[5] << 5 | self.output[6] << 6 | self.output[7] << 7 | self.output[8] << 8 | self.output[9] << 9 | self.output[10] << 10 | self.output[11] << 11 | self.output[12] << 12 | self.output[13] << 13 | self.output[14] << 14 | self.output[15] << 15
                    self.el2809.output = struct.pack('<H', output_bits)
                    self.ethercat.send_processdata()
                elif (input_bits >> 9) & 0x01:
                    self.output[8] = 0
                    self.output[6] = 0
                    output_bits = self.output[0] << 0 | self.output[1] << 1 | self.output[2] << 2 | self.output[3] << 3 | self.output[4] << 4 | self.output[5] << 5 | self.output[6] << 6 | self.output[7] << 7 | self.output[8] << 8 | self.output[9] << 9 | self.output[10] << 10 | self.output[11] << 11 | self.output[12] << 12 | self.output[13] << 13 | self.output[14] << 14 | self.output[15] << 15
                    self.el2809.output = struct.pack('<H', output_bits)
                    self.ethercat.send_processdata()
            
            if get_value(1, 19, 1)[0]:
                if (input_bits >> 7) & 0x01:
                    self.output[7] = 1
                    output_bits = self.output[0] << 0 | self.output[1] << 1 | self.output[2] << 2 | self.output[3] << 3 | self.output[4] << 4 | self.output[5] << 5 | self.output[6] << 6 | self.output[7] << 7 | self.output[8] << 8 | self.output[9] << 9 | self.output[10] << 10 | self.output[11] << 11 | self.output[12] << 12 | self.output[13] << 13 | self.output[14] << 14 | self.output[15] << 15
                    self.el2809.output = struct.pack('<H', output_bits)
                    self.ethercat.send_processdata()
                elif (input_bits >> 6) & 0x01:
                    self.output[7] = 0
                    output_bits = self.output[0] << 0 | self.output[1] << 1 | self.output[2] << 2 | self.output[3] << 3 | self.output[4] << 4 | self.output[5] << 5 | self.output[6] << 6 | self.output[7] << 7 | self.output[8] << 8 | self.output[9] << 9 | self.output[10] << 10 | self.output[11] << 11 | self.output[12] << 12 | self.output[13] << 13 | self.output[14] << 14 | self.output[15] << 15
                    self.el2809.output = struct.pack('<H', output_bits)
                    self.ethercat.send_processdata()
            else:
                if (input_bits >> 6) & 0x01:
                    self.output[7] = 1
                    output_bits = self.output[0] << 0 | self.output[1] << 1 | self.output[2] << 2 | self.output[3] << 3 | self.output[4] << 4 | self.output[5] << 5 | self.output[6] << 6 | self.output[7] << 7 | self.output[8] << 8 | self.output[9] << 9 | self.output[10] << 10 | self.output[11] << 11 | self.output[12] << 12 | self.output[13] << 13 | self.output[14] << 14 | self.output[15] << 15
                    self.el2809.output = struct.pack('<H', output_bits)
                    self.ethercat.send_processdata()
                elif (input_bits >> 7) & 0x01:
                    self.output[7] = 0
                    output_bits = self.output[0] << 0 | self.output[1] << 1 | self.output[2] << 2 | self.output[3] << 3 | self.output[4] << 4 | self.output[5] << 5 | self.output[6] << 6 | self.output[7] << 7 | self.output[8] << 8 | self.output[9] << 9 | self.output[10] << 10 | self.output[11] << 11 | self.output[12] << 12 | self.output[13] << 13 | self.output[14] << 14 | self.output[15] << 15
                    self.el2809.output = struct.pack('<H', output_bits)
                    self.ethercat.send_processdata()

            if (input_bits >> 10) & 0x01 or get_value(1, 28, 1)[0] == 1 or get_value(1, 1, 1)[0] == 1: #START
                self.akselerasi = False
                set_value(1, 28, [0])
                set_value(1, 1, [0])
                set_value(2, 7, [1])
            if (input_bits >> 0) & 0x01 or get_value(1, 29, 1)[0] == 1 or get_value(1, 0, 1)[0] == 1: #STOP
                self.akselerasi = False
                set_value(1, 29, [0])
                set_value(1, 0, [0])
                set_value(2, 7, [0])
            if (input_bits >> 1) & 0x01 or get_value(1, 30, 1)[0] == 1: #RESET
                set_value(2, 5, [0, 0])
                set_value(2, 7, [0]) # START 
                set_value(1, 2, [1]) # EMERGENCY BUTTON
                set_value(1, 6, [0]) # BUMPER
                set_value(2, 8, [0]) # E-STOP ERROR
                set_value(3, 221, [0]) # ID RFID
                self.indicator('OFF') # INDICATOR OFF
                self.buzzer_selector(0)
            if get_value(1, 27, 1)[0] == 1:
                save_config()
                set_value(1, 27, [0])
            if get_value(1, 26, 1)[0] == 1:
                set_RFID()
                set_value(1, 26, [0])
            
            if (input_bits >> 5) & 0x01 == 1 and get_value(2, 7, 1)[0] == 1:
                self.canbus.set_kecepatan_motor([0, 0])
                set_value(1, 6, [1])
                set_value(2, 8, [1])
            
            if self.canbus.voltage <= registers_to_float32(get_value(3, 1012, 2)):
                set_value(2, 10, [1])
            
            self.route(get_value(1, 20, 5))

            if get_value(2, 9, 1)[0] == 1:
                self.indicator('R')
                self.buzzer_selector(2)
                self.canbus.set_kecepatan_motor([0, 0])
                set_value(2, 7, [0])
                set_value(2, 5, [0, 0])
            elif get_value(2, 10, 1)[0] == 1:
                self.indicator('R')
                self.buzzer_selector(2)
                self.canbus.set_kecepatan_motor([0, 0])
                set_value(2, 7, [0])
                set_value(2, 5, [0, 0])
            elif get_value(2, 8, 1)[0] == 1:
                self.indicator('R')
                self.buzzer_selector(2)
                self.canbus.set_kecepatan_motor([0, 0])
                set_value(2, 7, [0])
                set_value(2, 5, [0, 0])
            elif (input_bits >> 3) & 0x01 == 0:
                self.indicator('R')
                self.buzzer_selector(0)
                self.canbus.set_kecepatan_motor([0, 0])
                set_value(2, 7, [0])
                set_value(2, 5, [0, 0])
            elif (self.canbus.flag >> 1) & 1 == 0 and get_value(2, 7, 1)[0] == 0 and get_value(2, 8, 1)[0] == 0:
                self.indicator('Y')
                self.buzzer_selector(0)
                self.canbus.set_kecepatan_motor([0, 0])
                set_value(2, 5, [0, 0])
            else:
                if get_value(2, 7, 1)[0] == 1 and get_value(2, 8, 1)[0] == 0:

                    if self.temp_SPEED != get_value(3, 40, 1)[0]:
                        self.akselerasi = False
                        self.temp_SPEED = get_value(3, 40, 1)[0]

                    if self.akselerasi == False:
                        self.previous_time = time.time() * 1000
                        self.akselerasi = True

                    self.select_track(self.track_select)
                    set_value(2, 3, [1])
                    if get_value(1, 25, 1)[0] == 1:
                        if (input_bits >> 11) & 0x01 == 1:
                            set_value(2, 5, [0, 0])
                            if (input_bits >> 4) & 0x01 == 0:
                                set_value(2, 5, [1])
                                self.indicator('Y')
                                self.buzzer_selector(1)
                                target_speed = [int(speed(20) + self.pid_slow.compute(self.error)), int(-speed(20) + self.pid_slow.compute(self.error))]
                                self.canbus.set_kecepatan_motor(self.update_speed_slow(target_speed))
                                self.buzzer = True
                                self.akselerasi = False
                            else:
                                self.indicator('G')
                                self.buzzer_selector(self.temp_buzzer)
                                set_value(2, 5, [0, 0])
                                target_speed = [int(speed(get_value(3, 40, 1)[0]) + self.pid.compute(self.error)), int(-speed(get_value(3, 40, 1)[0]) + self.pid.compute(self.error))]
                                if time.time() * 1000 - self.previous_time <= self.interval:
                                    self.canbus.set_kecepatan_motor(self.update_speed(target_speed))
                                else:
                                    self.current_speed = target_speed
                                    self.canbus.set_kecepatan_motor(target_speed)
                        else:
                            self.indicator('R')
                            self.buzzer_selector(2)
                            set_value(2, 5, [0, 1])
                            self.canbus.set_kecepatan_motor([int(self.pid.compute(self.error)), int(self.pid.compute(self.error))])
                            self.current_speed = [0, 0]
                            self.buzzer = True
                            self.akselerasi = False
                    else:
                        self.indicator('G')
                        self.buzzer_selector(self.temp_buzzer)
                        set_value(2, 5, [0, 0])
                        target_speed = [int(speed(get_value(3, 40, 1)[0]) + self.pid.compute(self.error)), int(-speed(get_value(3, 40, 1)[0]) + self.pid.compute(self.error))]
                        if time.time() * 1000 - self.previous_time <= self.interval:
                            self.canbus.set_kecepatan_motor(self.update_speed(target_speed))
                        else:
                            self.current_speed = target_speed
                            self.canbus.set_kecepatan_motor(target_speed)

                    if self.buzzer == 1 and (input_bits >> 11) & 0x01 == 1 and (input_bits >> 4) & 0x01 == 1:
                        self.indicator('OFF')
                        self.buzzer_selector(self.temp_buzzer)
                        self.buzzer = False
                        
                    # if (self.canbus.flag >> 1) & 1 == 0:
                    #     self.indicator('R')
                    #     self.buzzer_selector(1)
                    #     self.current_speed = [0, 0]
                    #     set_value(2, 8, [1])
                else:
                    self.indicator('OFF')
                    self.buzzer_selector(0)
                    if self.akselerasi == False:
                        self.previous_time = time.time() * 1000
                        self.akselerasi = True
                    set_value(2, 3, [0])
                    target_speed = [0, 0]
                    if time.time() * 1000 - self.previous_time <= self.interval:
                        self.canbus.set_kecepatan_motor(self.update_speed(target_speed))
                    else:
                        self.current_speed = target_speed
                        self.canbus.set_kecepatan_motor(target_speed)

            output_bits = self.output[0] << 0 | self.output[1] << 1 | self.output[2] << 2 | self.output[3] << 3 | self.output[4] << 4 | self.output[5] << 5 | self.output[6] << 6 | self.output[7] << 7 | self.output[8] << 8 | self.output[9] << 9 | self.output[10] << 10 | self.output[11] << 11 | self.output[12] << 12 | self.output[13] << 13 | self.output[14] << 14 | self.output[15] << 15
            self.el2809.output = struct.pack('<H', output_bits)
            self.ethercat.send_processdata()
            await asyncio.sleep(0.0001)
                       
    def select_track(self, track : int):
        if track == 2:
            self.error = self.canbus.sensor[2]
        elif track == 1:
            self.error = self.canbus.sensor[0]
        else:
             self.error = self.canbus.sensor[1]

    def route(self, value : list[int]):
        if value[0] == 1:
            add_or_update_data(get_value(3, 35, 1)[0], get_value(3, 37, 1)[0])
            set_value(1, 20, [0])
        if value[1] == 1:
            delete_data(get_value(3, 35, 1)[0], get_value(3, 37, 1)[0])
            set_value(1, 21, [0])
        if value[2] == 1:
            insert_data(get_value(3, 35, 1)[0], get_value(3, 36, 1)[0])
        if value[3] == 1:
            query_data(get_value(3, 35, 1)[0], get_value(3, 37, 1)[0])
        if value[4] == 1:
            query_route(get_value(3, 35, 1)[0])
    
    def update_speed(self, target_speed):
        steps = int(self.interval / self.acceleration_rate) 
        for i in range(2):
            speed_step = int(abs(target_speed[i] - self.current_speed[i]) / steps)
            if self.current_speed[i] < target_speed[i]:
                self.current_speed[i] = min(self.current_speed[i] + speed_step, target_speed[i])
            elif self.current_speed[i] > target_speed[i]:
                self.current_speed[i] = max(self.current_speed[i] - speed_step, target_speed[i])
        return self.current_speed
    
    def update_speed_slow(self, target_speed):
        new_speed = [0, 0]
        for i in range(2):
            if self.current_speed[i] < target_speed[i]:
                new_speed[i] = min(self.current_speed[i] + self.acceleration_rate, target_speed[i])
            elif self.current_speed[i] > target_speed[i]:
                new_speed[i] = max(self.current_speed[i] - self.acceleration_rate, target_speed[i])
            else:
                new_speed[i] = self.current_speed[i]
        self.current_speed = new_speed
        return new_speed
    
    def buzzer_selector(self, value : int):
        if value == 1:
            self.output[2] = 1
            self.output[3] = 0
            self.output[4] = 0
            self.output[5] = 0
        elif value == 2:
            self.output[2] = 0
            self.output[3] = 1
            self.output[4] = 0
            self.output[5] = 0
        elif value == 3:
            self.output[2] = 0
            self.output[3] = 0
            self.output[4] = 1
            self.output[5] = 0
        elif value == 4:
            self.output[2] = 0
            self.output[3] = 0
            self.output[4] = 0
            self.output[5] = 1
        else:
            self.output[2] = 0
            self.output[3] = 0
            self.output[4] = 0
            self.output[5] = 0

    def indicator(self, data : str):
        if data == 'R':
            self.output[10] = 1
            self.output[1] = 0
            self.output[0] = 0
        elif data == 'Y':
            self.output[10] = 0
            self.output[1] = 1
            self.output[0] = 0
        elif data == 'G':
            self.output[10] = 0
            self.output[1] = 0
            self.output[0] = 1
        else:
            self.output[10] = 0
            self.output[1] = 0
            self.output[0] = 0
    
    async def main(self):
        await asyncio.gather(self.main_robot(), self.run_robot())
    

if __name__ == "__main__":
    app = robot()

    Thread(target=start_modbus_server_serial, args=(context, identity, serial_kwargs), daemon=True).start()
    Thread(target=start_modbus_server_serial, args=(context, identity, serial_kwargs_1), daemon=True).start()

    try:
        asyncio.run(app.main())

    except KeyboardInterrupt:
        print("Server Stopped")
        app.ethercat.close()
        exit()
    except Exception as e:
        print(f"Error: {e}")
        app.ethercat.close()
        exit()