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
    # ==============================================================================
    # ================================== Parameter ==================================
    # ==============================================================================
    error = 0
    previous_time = 0
    temp_buzzer = 0
    buzzer = False
    akselerasi = False
    temp_RFID = 0
    temp_SPEED = 0
    current_speed = [0, 0] 
    acceleration_rate = 35
    command = False
    bumper = False
    ssa__in = [True, True, True, False]
    lmp_out = [False, False, False]
    snd_out = [False, False, False, False] 
    # ==============================================================================
    # ==============================================================================
    # ==============================================================================
    def __init__(self):
        set_single_holding_register(slave_map_io['map_io']['dashboard']['speed']['addr'], config['Speed'])
        set_single_holding_register(slave_map_io['map_io']['setting']['idcar']['addr'], config['ID_CAR'])
        set_single_holding_register(slave_map_io['map_io']['setting']['accel']['addr'], config['Acceleration'])
        set_single_holding_register(slave_map_io['map_io']['setting']['music']['addr'], config['Default_Music'])
        set_single_holding_register(slave_map_io['map_io']['setting']['track']['addr'], config['Select_Track'])
        set_single_holding_register(slave_map_io['map_io']['setting']['obstacle']['addr'], config['Obstacle'])
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
    

    def motion_task_robot(self):
        while True:
            if self.canbus.data_RFID != self.temp_RFID:
                self.temp_RFID = self.canbus.data_RFID
                with open(file_path, "r") as node:
                    data_node = json.load(node)
                if str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr'])) in data_node:
                    if str(self.canbus.data_RFID) in data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))]:
                        for i in range(len(data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)])):
                            set_value(slave_map_io['map_io']['dashboard']['state']['in_progress']['fc'], slave_map_io['map_io']['dashboard']['state']['in_progress']['addr'], [1])
                            # ============= FORWARD =============
                            if data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'forward':
                                self.interval = data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]
                                self.command = True
                            # ============= STOP =============
                            elif data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'stop':
                                self.interval = data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]
                                self.command = False
                            # ============= MUSIC =============
                            elif data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'music':
                                self.temp_buzzer = data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]
                            # ============= STICK =============
                            elif data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'stick':
                                set_value(slave_map_io['map_io']['dashboard']['stick']['fc'], slave_map_io['map_io']['dashboard']['stick']['addr'], [data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]])
                            # ============= SPEED =============
                            elif data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'speed':
                                set_value(slave_map_io['map_io']['dashboard']['speed']['fc'], slave_map_io['map_io']['dashboard']['speed']['addr'], [data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]])
                                self.akselerasi = False
                            # ============= OBSTACLE =============
                            elif data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'obs set':
                                set_value(slave_map_io['map_io']['setting']['obstacle']['fc'], slave_map_io['map_io']['setting']['obstacle']['addr'], [data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]])
                            # ============= ROUTE TRANS =============
                            elif data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'route trans':
                                set_value(slave_map_io['map_io']['dasbboard']['route']['fc'], slave_map_io['map_io']['dashboard']['route']['addr'], [data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]])
                                break
                            # ============= TURN =============
                            elif data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'turn':
                                self.track_select = data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]
                            # ============= DELAY =============
                            elif data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'delay':
                                time.sleep(data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']['addr']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1])
                        set_value(slave_map_io['map_io']['dashboard']['state']['in_progress']['fc'], slave_map_io['map_io']['dashboard']['state']['in_progress']['addr'], [0])
            time.sleep(0.01)

    def io_plc(self):
        conn = ModbusTcpClient(host=io_map['modbus']['TCP']['host'], port=io_map['modbus']['TCP']['port'])
        while True:
            if conn.connected:
                # ======================================
                # ================= LAMP ======================================
                # =======================================================
                indicator(conn, self.lmp_out)
                # ======================================
                # ================= SOUND ======================================
                # =======================================================
                sound(conn, self.snd_out)
                # ======================================
                # ================= SENSOR ALARM ======================================
                # =======================================================
                self.ssa__in[0] = emergency_stop(conn)
                self.ssa__in[1] = Obstacle_warning(conn)
                self.ssa__in[2] = obstacle_stop(conn)
                self.ssa__in[3] = bumper(conn)
                set_single_discrete_input(slave_map_io['map_io']['dashboard']['alarm']['estop_indicator']['addr'], emergency_stop(conn))
                # ======================================
                # ================= WHEEL & HOOK CONTROL ======================================
                # ================= WHEEL ======================================
                if button_wheel(conn) == 0 and wheel_up(conn) and wheel_down(conn) == 0:
                    set_single_coil(slave_map_io['map_io']['dashboard']['control']['wheel']['addr'], 1)
                if button_wheel(conn) == 0 and wheel_up(conn) == 0 and wheel_down(conn):
                    set_single_coil(slave_map_io['map_io']['dashboard']['control']['wheel']['addr'], 0)
                wheel_c(conn, get_single_coil(slave_map_io['map_io']['dashboard']['control']['wheel']['addr']))  
                set_multiple_discrete_inputs(slave_map_io['map_io']['dashboard']['control']['state']['wheel_up']['addr'], [wheel_up(conn), wheel_down(conn)])
                # ======================================
                # ================= WHEEL & HOOK CONTROL ======================================
                # ================= HOOK ======================================
                hook_c(conn, get_single_coil(slave_map_io['map_io']['dashboard']['control']['hook']['addr']))  
                set_multiple_discrete_inputs(slave_map_io['map_io']['dashboard']['control']['state']['hook_up']['addr'], [hook_up(conn), hook_down(conn)])
                # ======================================
                # ================= START & STOP & RESET ======================================
                # =======================================================
                if start(conn) == 1 or get_single_coil(slave_map_io['map_io']['dashboard']['control']['start']['addr']) == 1:
                    self.akselerasi = False
                    self.command = True
                elif stop(conn) == 1 or get_single_coil(slave_map_io['map_io']['dashboard']['control']['stop']['addr']) == 1:
                    self.akselerasi = False
                    self.command = False
                elif reset(conn) == 1 or get_single_coil(slave_map_io['map_io']['dashboard']['control']['reset']['addr']) == 1:
                    self.bumper = False
                    self.command = False
                # ======================================
                # ================= START & STOP & RESET ======================================
                # =======================================================
            time.sleep(0.01)


    
    def main_robot(self):
        while True:
            route()
            self.canbus.read_data_sensor()
            set_single_discrete_input(slave_map_io['map_io']['dashboard']['rfid']['addr'], self.canbus.data_RFID)
            set_multiple_input_registers(slave_map_io['map_io']['dashboard']['clock']['hour']['addr'], get_uptime())

            if self.ssa__in[3] == True:
                self.bumper = True
            
            if self.bumper == True:
                self.canbus.set_kecepatan_motor([0, 0])
                self.snd_out = [0, 1, 0, 0]
                self.lmp_out = [1, 0, 0]
            else:
                
                print((self.canbus.flag >> 1) & 1)
                if self.ssa__in[0] != 1:
                    self.commnad = False
                    self.canbus.set_kecepatan_motor([0, 0])
                    self.lmp_out = [1, 0, 0]
                elif self.ssa__in[0] == 1:
                    if self.command == 1:
                        if self.temp_SPEED != get_single_holding_register(slave_map_io['map_io']['dashboard']['speed']['addr']):
                            self.akselerasi = False
                            self.temp_SPEED = get_single_holding_register(slave_map_io['map_io']['dashboard']['speed']['addr'])

                        if self.akselerasi == False:
                            self.previous_time = time.time() * 1000
                            self.akselerasi = True

                        self.select_track(self.track_select)
                        set_single_discrete_input(slave_map_io['map_io']['dashboard']['state']['forward']['addr'], 1)
                        if get_single_holding_register(slave_map_io['map_io']['setting']['obstacle']['addr']) != 0:
                            if self.ssa__in[2] == 1:
                                if self.ssa__in[1] == 0:
                                    self.snd_out = [1, 0, 0, 0]
                                    self.lmp_out = [0, 0, 1]
                                    set_multiple_discrete_inputs(slave_map_io['map_io']['dashboard']['alarm']['slow_area']['addr'], [1, 0])
                                    target_speed = [int(speed(20) + self.pid_slow.compute(self.error)), int(-speed(20) + self.pid_slow.compute(self.error))]
                                    self.canbus.set_kecepatan_motor(self.update_speed_slow(target_speed))
                                    self.buzzer = True
                                    self.akselerasi = False
                                else:
                                    self.snd_out = [0, 0, 0, 1]
                                    self.lmp_out = [0, 1, 0]
                                    set_multiple_discrete_inputs(slave_map_io['map_io']['dashboard']['alarm']['slow_area']['addr'], [0, 0])
                                    target_speed = [int(speed(get_single_holding_register(slave_map_io['map_io']['dashboard']['speed']['addr'])) + self.pid.compute(self.error)), int(-speed(get_single_holding_register(slave_map_io['map_io']['dashboard']['speed']['addr'])) + self.pid.compute(self.error))]
                                    if time.time() * 1000 - self.previous_time <= self.interval:
                                        self.canbus.set_kecepatan_motor(self.update_speed(target_speed))
                                    else:
                                        self.current_speed = target_speed
                                        self.canbus.set_kecepatan_motor(target_speed)
                            else:
                                self.snd_out = [0, 1, 0, 0]
                                self.lmp_out = [1, 0, 0]
                                set_multiple_discrete_inputs(slave_map_io['map_io']['dashboard']['alarm']['slow_area']['addr'], [0, 1])
                                self.canbus.set_kecepatan_motor([int(self.pid.compute(self.error)), int(self.pid.compute(self.error))])
                                self.current_speed = [0, 0]
                                self.buzzer = True
                                self.akselerasi = False
                        else:
                            self.lmp_out = [0, 1, 0]
                            set_multiple_discrete_inputs(slave_map_io['map_io']['dashboard']['alarm']['slow_area']['addr'], [0, 0])
                            target_speed = [int(speed(get_single_holding_register(slave_map_io['map_io']['dashboard']['speed']['addr'])) + self.pid.compute(self.error)), int(-speed(get_single_holding_register(slave_map_io['map_io']['dashboard']['speed']['addr'])) + self.pid.compute(self.error))]
                            if time.time() * 1000 - self.previous_time <= self.interval:
                                self.canbus.set_kecepatan_motor(self.update_speed(target_speed))
                            else:
                                self.current_speed = target_speed
                                self.canbus.set_kecepatan_motor(target_speed) 
                                
                        # if (self.canbus.flag >> 1) & 1 == 0:
                        #     indicator(conn, 'R')
                        #     self.current_speed = [0, 0]
                        #     set_single_discrete_input(slave_map_io['map_io']['dashboard']['state']['out_of_track']['addr'], 1)

                    else:
                        self.snd_out = [0, 0, 0, 0]
                        self.lmp_out = [0, 0, 0]
                        if self.akselerasi == False:
                            self.previous_time = time.time() * 1000
                            self.akselerasi = True
                        set_single_discrete_input(slave_map_io['map_io']['dashboard']['state']['forward']['addr'], 0)
                        target_speed = [0, 0]
                        if time.time() * 1000 - self.previous_time <= self.interval:
                            self.canbus.set_kecepatan_motor(self.update_speed(target_speed))
                        else:
                            self.current_speed = target_speed
                            self.canbus.set_kecepatan_motor(target_speed)
            data = [
                get_cpu_temperature(), 
                self.canbus.temp_driver, 
                self.canbus.error[1], 
                self.canbus.error[0], 
                self.canbus.current[1], 
                self.canbus.current[0], 
                0, 
                self.canbus.can_open, 
                self.bumper
            ]
            status(data)
            # await asyncio.sleep(0.0001)

    # async def main(self):
    #     await asyncio.gather(self.main_robot(), self.run_robot())

if __name__ == "__main__":
    app = robot()
    Thread(target=app.motion_task_robot, daemon=True).start()
    Thread(target=app.io_plc, daemon=True).start()
    Thread(target=start_modbus_server_serial, args=(context, identity, slave_map_io['modbus']['RTU']), daemon=True).start()
    try:
        app.main_robot()
    except KeyboardInterrupt:
        print("Stop")
        exit()
    except Exception as e:
        print(f"Error {e}")
