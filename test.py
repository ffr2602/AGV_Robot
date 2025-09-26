import asyncio
from asyncio import threads
from module.database import *
from module.konversi import *
from module.can_bus import CAN_setting
from module.pid import PID
import json
import time
import struct

conn = ModbusTcpClient(host=io_map['modbus']['TCP']['host'], port=io_map['modbus']['TCP']['port'])
class robot():

    # Parameter ==================================
    error = 0
    temp_buzzer = 0
    buzzer = False
    akselerasi = False
    temp_RFID = 0
    temp_SPEED = 0

    current_speed = [0, 0] 
    acceleration_rate = 35

    wheel_temp = False

    previous_time = 0

    command = False
    bumper = False
    

    # ============================================
    def __init__(self):
        set_value(slave_map_io['map_io']['setting']['idcar']['fc'], slave_map_io['map_io']['setting']['idcar']['addr'], [config['ID_CAR']])
        set_value(slave_map_io['map_io']['setting']['accel']['fc'], slave_map_io['map_io']['setting']['accel']['addr'], [config['Acceleration']])
        set_value(slave_map_io['map_io']['setting']['music']['fc'], slave_map_io['map_io']['setting']['music']['addr'], [config['Default_Music']])
        set_value(slave_map_io['map_io']['setting']['track']['fc'], slave_map_io['map_io']['setting']['track']['addr'], [config['Select_Track']])
        set_value(slave_map_io['map_io']['setting']['obstacle']['fc'], slave_map_io['map_io']['setting']['obstacle']['addr'], [config['Obstacle']])
        self.temp_buzzer = config['Default_Music']
        self.interval = config['Acceleration']
        self.track_select = config['Select_Track']
        self.pid = PID(config['P'], config['I'], config['D'])
        self.pid_slow = PID(9.00, 0.00, 0.00)
        self.canbus = CAN_setting()
    
    
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
    

    def select_track(self, track : int):
        if track == 2:
            self.error = self.canbus.sensor[2]
        elif track == 1:
            self.error = self.canbus.sensor[0]
        else:
            self.error = self.canbus.sensor[1]
    

    async def run_robot(self):
        while True:
            if self.canbus.data_RFID != self.temp_RFID:
                self.temp_RFID = self.canbus.data_RFID
                with open(file_path, "r") as node:
                    data_node = json.load(node)
                if str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr'])) in data_node:
                    if str(self.canbus.data_RFID) in data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))]:
                        for i in range(len(data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)])):
                            set_value(slave_map_io['map_io']['dashboard']['state']['in_progress']['fc'], slave_map_io['map_io']['dashboard']['state']['in_progress']['addr'], [1])
                            # ============= FORWARD 
                            if data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'forward':
                                self.interval = data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]
                                self.command = True
                            # ============= STOP
                            elif data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'stop':
                                self.interval = data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]
                                self.command = False
                            # ============= MUSIC
                            elif data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'music':
                                self.temp_buzzer = data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]
                            # ============= STICK
                            elif data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'stick':
                                set_value(slave_map_io['map_io']['dashboard']['stick']['fc'], slave_map_io['map_io']['dashboard']['stick']['addr'], [data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]])
                            # ============= SPEED
                            elif data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'speed':
                                set_value(slave_map_io['map_io']['dashboard']['speed']['fc'], slave_map_io['map_io']['dashboard']['speed']['addr'], [data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]])
                                self.akselerasi = False
                            # ============= OBSTACLE
                            elif data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'obs set':
                                set_value(slave_map_io['map_io']['setting']['obstacle']['fc'], slave_map_io['map_io']['setting']['obstacle']['addr'], [data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]])
                            # ============= ROUTE TRANS
                            elif data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'route trans':
                                set_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr'], [data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]])
                                break
                            # ============= TURN
                            elif data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'turn':
                                self.track_select = data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]
                            # ============= DELAY
                            elif data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'delay':
                                await asyncio.sleep(data_node[str(get_single_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1])
                        set_value(slave_map_io['map_io']['dashboard']['state']['in_progress']['fc'], slave_map_io['map_io']['dashboard']['state']['in_progress']['addr'], [0])
            await asyncio.sleep(0.001)
    
    async def main_robot(self):
        while True:
            route()
            self.canbus.read_data_sensor()
            set_value(slave_map_io['map_io']['dashboard']['rfid']['fc'], slave_map_io['map_io']['dashboard']['rfid']['addr'], [self.canbus.data_RFID])
            # ================= Wheel & Hook Control 
            # ================= Button
            if button_wheel(conn) == 0 and wheel_up(conn) and wheel_down(conn) == 0:
                set_single_coil_X0(slave_map_io['map_io']['dashboard']['control']['wheel']['addr'], 1)
            if button_wheel(conn) == 0 and wheel_up(conn) == 0 and wheel_down(conn):
                set_single_coil_X0(slave_map_io['map_io']['dashboard']['control']['wheel']['addr'], 0)
            wheel_c(conn, get_single_coil_X0(slave_map_io['map_io']['dashboard']['control']['wheel']['addr']))  
            set_multiple_coil_X1(slave_map_io['map_io']['dashboard']['control']['state']['wheel_up']['addr'], [wheel_up(conn), wheel_down(conn)])
            # ======================================
            # ================= START & STOP & RESET
            if start(conn) == 1 or get_single_coil_X0(slave_map_io['map_io']['dashboard']['control']['start']['addr']) == 1:
                self.command = True
            elif stop(conn) == 1 or get_single_coil_X0(slave_map_io['map_io']['dashboard']['control']['stop']['addr']) == 1:
                self.command = False
            elif reset(conn) == 1 or get_single_coil_X0(slave_map_io['map_io']['dashboard']['control']['reset']['addr']) == 1:
                print(reset(conn))
            # =================
            if emergency_stop(conn=conn) != 1:
                self.commnad = False
                self.canbus.set_kecepatan_motor([0, 0])
                indicator(conn, data="R")
            # elif emergency_stop(conn=conn) == 0:
            #     if self.command == 1:
            #         if self.temp_SPEED != get_value(3, 40, 1)[0]:
            #             self.akselerasi = False
            #             self.temp_SPEED = get_value(3, 40, 1)[0]

            #         if self.akselerasi == False:
            #             self.previous_time = time.time() * 1000
            #             self.akselerasi = True

            #         self.select_track(self.track_select)
            #         set_value(2, 3, [1])
            #         if get_single_value(slave_map_io['map_io']['setting']['obstacle']['fc'], slave_map_io['map_io']['setting']['obstacle']['addr']) != 0:
            #             if obstacle_stop(conn) == 1:
            #                 set_value(2, 5, [0, 0])
            #                 if Obstacle_warning(conn) == 0:
            #                     set_value(2, 5, [1])
            #                     indicator(conn, 'Y')
            #                     self.buzzer_selector(1)
            #                     target_speed = [int(speed(20) + self.pid_slow.compute(self.error)), int(-speed(20) + self.pid_slow.compute(self.error))]
            #                     self.canbus.set_kecepatan_motor(self.update_speed_slow(target_speed))
            #                     self.buzzer = True
            #                     self.akselerasi = False
            #                 else:
            #                     self.indicator('G')
            #                     self.buzzer_selector(self.temp_buzzer)
            #                     set_value(2, 5, [0, 0])
            #                     target_speed = [int(speed(get_value(3, 40, 1)[0]) + self.pid.compute(self.error)), int(-speed(get_value(3, 40, 1)[0]) + self.pid.compute(self.error))]
            #                     if time.time() * 1000 - self.previous_time <= self.interval:
            #                         self.canbus.set_kecepatan_motor(self.update_speed(target_speed))
            #                     else:
            #                         self.current_speed = target_speed
            #                         self.canbus.set_kecepatan_motor(target_speed)
            #             else:
            #                 self.indicator('R')
            #                 self.buzzer_selector(2)
            #                 set_value(2, 5, [0, 1])
            #                 self.canbus.set_kecepatan_motor([int(self.pid.compute(self.error)), int(self.pid.compute(self.error))])
            #                 self.current_speed = [0, 0]
            #                 self.buzzer = True
            #                 self.akselerasi = False
            # else:
            #     indicator(conn, data="OFF")
            data = [
                get_cpu_temperature(), 
                self.canbus.temp_driver, 
                self.canbus.error[1], 
                self.canbus.error[0], 
                self.canbus.current[1], 
                self.canbus.current[0], 
                conn.connected, 
                self.canbus.can_open, 
                self.bumper
            ]
            status(data)
            await asyncio.sleep(0.0001)

    async def main(self):
        await asyncio.gather(self.main_robot(), self.run_robot())

if __name__ == "__main__":
    app = robot()
    Thread(target=start_modbus_server_serial, args=(context, identity, slave_map_io['modbus']['RTU']), daemon=True).start()
    try:
        asyncio.run(app.main())
    except KeyboardInterrupt:
        print("Stop")
        exit()
    except Exception as e:
        print(f"Error {e}")
