from module.database import *
from module.konversi import *
from module.can_bus import CAN_setting
from module.pid import PID
from threading import Thread
import json
import time


class robot():
    # ==============================================================================
    # ================================== Parameter ==================================
    # ==============================================================================
    error = 0
    previous_time = 0
    # ======== Voltage | Time ========
    batt = [0.0, None]
    # ======== Speed Left | Speed Right ========
    ss__spd = [0, 0]
    # ======== Music | RFID | Speed | Acceleration Rate ========
    tmp_com = [0, 0, 0, 35]
    # ======== Alarm | Command | Acceleration | Bumper ========
    cmd_com = [False, False, False, False]
    # ======== TCP | CAN ========
    cc___er = [True, True]
    # ======== E-Stop Button | Obstacle Warn | Obstacle Stop | Bumper | E-STOP Command ========
    ssa__in = [True, True, True, False, False]
    # ======== Red | Green | Yellow ========
    lmp_out = [False, False, False]
    # ======== S1 | S2 | S3 | S4 ========
    snd_out = [False, False, False, False] 
    # ==============================================================================
    # ==============================================================================
    # ==============================================================================
    def __init__(self):
        set_single_holding_register(slave_map_io['map_io']['dashboard']['speed'], config['Speed'])
        set_single_holding_register(slave_map_io['map_io']['setting']['idcar'], config['ID_CAR'])
        set_single_holding_register(slave_map_io['map_io']['setting']['accel'], config['Acceleration'])
        set_single_holding_register(slave_map_io['map_io']['setting']['music'], config['Default_Music'])
        set_single_holding_register(slave_map_io['map_io']['setting']['track'], config['Select_Track'])
        set_single_holding_register(slave_map_io['map_io']['setting']['obstacle'], config['Obstacle'])
        self.interval = config['Acceleration']
        self.pid = PID(config['P'], config['I'], config['D'])
        self.pid_slow = PID(9.00, 0.00, 0.00)
        self.canbus = CAN_setting()
    
    def update_speed(self, target_speed):
        steps = int(self.interval / self.tmp_com[3]) 
        for i in range(2):
            speed_step = int(abs(target_speed[i] - self.ss__spd[i]) / steps)
            if self.ss__spd[i] < target_speed[i]:
                self.ss__spd[i] = min(self.ss__spd[i] + speed_step, target_speed[i])
            elif self.ss__spd[i] > target_speed[i]:
                self.ss__spd[i] = max(self.ss__spd[i] - speed_step, target_speed[i])
        return self.ss__spd

    def update_speed_slow(self, target_speed):
        new_speed = [0, 0]
        for i in range(2):
            if self.ss__spd[i] < target_speed[i]:
                new_speed[i] = min(self.ss__spd[i] + self.tmp_com[3], target_speed[i])
            elif self.ss__spd[i] > target_speed[i]:
                new_speed[i] = max(self.ss__spd[i] - self.tmp_com[3], target_speed[i])
            else:
                new_speed[i] = self.ss__spd[i]
        self.ss__spd = new_speed
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
            if self.canbus.data_RFID != self.tmp_com[1]:
                self.tmp_com[1] = self.canbus.data_RFID
                with open(file_path, "r") as node:
                    data_node = json.load(node)
                if str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route'])) in data_node:
                    if str(self.canbus.data_RFID) in data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))]:
                        for i in range(len(data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)])):
                            set_single_discrete_input(slave_map_io['map_io']['dashboard']['state']['in_progress'], True)
                            # ============= FORWARD =============
                            if data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'forward':
                                self.interval = data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]
                                self.cmd_com[1] = True
                            # ============= STOP =============
                            elif data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'stop':
                                self.interval = data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]
                                self.cmd_com[1] = False
                            # ============= E-STOP =============
                            elif data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'e-stop':
                                self.ssa__in[4] = True
                            # ============= SOUND =============
                            elif data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'sound':
                                self.tmp_com[0] = data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1]
                            # ============= STICK =============
                            elif data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'stick':
                                set_single_coil(slave_map_io['map_io']['dashboard']['stick'], data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1])
                            # ============= SPEED =============
                            elif data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'speed':
                                set_single_holding_register(slave_map_io['map_io']['dashboard']['speed'], data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1])
                                self.cmd_com[2] = False
                            # ============= OBSTACLE =============
                            elif data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'obs-set':
                                set_single_holding_register(slave_map_io['map_io']['setting']['obstacle'], data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1])
                            # ============= ROUTE TRANS =============
                            elif data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'route-trans':
                                set_single_holding_register(slave_map_io['map_io']['dashboard']['route'], data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1])
                                break
                            # ============= TURN =============
                            elif data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'track':
                                set_single_holding_register(slave_map_io['map_io']['setting']['track'], data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1])
                            # ============= DELAY =============
                            elif data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][0] == 'delay':
                                time.sleep(data_node[str(get_single_holding_register(slave_map_io['map_io']['dashboard']['route']))][str(self.canbus.data_RFID)][f'motion_{i + 1}'][1])
                        set_single_discrete_input(slave_map_io['map_io']['dashboard']['state']['in_progress'], False)
            time.sleep(0.01)

    def io_plc(self):
        conn = ModbusTcpClient(host=io_map['modbus']['TCP']['host'], port=io_map['modbus']['TCP']['port'], timeout=1)
        while True:
            self.cc___er[0] = conn.connected
            try:
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
                    set_single_discrete_input(slave_map_io['map_io']['dashboard']['alarm']['estop_indicator'], emergency_stop(conn))
                    # ======================================
                    # ================= WHEEL & HOOK CONTROL ======================================
                    # ================= WHEEL ======================================
                    if button_wheel(conn) == 0 and wheel_up(conn) and wheel_down(conn) == 0:
                        set_single_coil(slave_map_io['map_io']['dashboard']['control']['wheel'], 1)
                    if button_wheel(conn) == 0 and wheel_up(conn) == 0 and wheel_down(conn):
                        set_single_coil(slave_map_io['map_io']['dashboard']['control']['wheel'], 0)
                    wheel_c(conn, get_single_coil(slave_map_io['map_io']['dashboard']['control']['wheel']))  
                    set_multiple_discrete_inputs(slave_map_io['map_io']['dashboard']['control']['state']['wheel_up'], [wheel_up(conn), wheel_down(conn)])
                    # ======================================
                    # ================= WHEEL & HOOK CONTROL ======================================
                    # ================= HOOK ======================================
                    hook_c(conn, get_single_coil(slave_map_io['map_io']['dashboard']['control']['hook']))  
                    set_multiple_discrete_inputs(slave_map_io['map_io']['dashboard']['control']['state']['hook_up'], [hook_up(conn), hook_down(conn)])
                    # ======================================
                    # ================= START & STOP & RESET ======================================
                    # =======================================================
                    if start(conn) == 1 or get_single_coil(slave_map_io['map_io']['dashboard']['control']['start']) == 1:
                        self.cmd_com[2] = False
                        self.cmd_com[1] = True
                    elif stop(conn) == 1 or get_single_coil(slave_map_io['map_io']['dashboard']['control']['stop']) == 1:
                        self.cmd_com[2] = False
                        self.cmd_com[1] = False
                    elif reset(conn) == 1 or get_single_coil(slave_map_io['map_io']['dashboard']['control']['reset']) == 1:
                        self.cmd_com[3] = False
                        self.cmd_com[1] = False
                        self.ssa__in[4] = False
                        self.canbus.data_RFID = 0
                    # ======================================
                    # =======================================================
                    # =======================================================
            except Exception:
                conn.close()
            time.sleep(0.01)
    
    def main_robot(self):
        while True:
            route()
            self.canbus.read_data_sensor()
            # ======================================
            # ================= ALARM FAULT ======================================
            # =======================================================
            self.batt[0] = self.canbus.voltage - 0.22
            self.cc___er[1] = self.canbus.can_open
            if not self.cc___er[1]:
                self.batt[0] = 0.0
                self.canbus.flag = 2
            if not self.cc___er[0] or not self.cc___er[1] or self.canbus.error[0] != 0 or self.canbus.error[1] != 0:
                self.cmd_com[0] = True
            elif self.batt[0] <= config['Low_Voltage']:
                if self.batt[1] is None:
                    self.batt[1] = time.time() * 1000
                elif time.time() * 1000 - self.batt[1] >= 5000:
                    set_single_input_register(slave_map_io['map_io']['status']['battery_indicator'], 2)
                    self.cmd_com[0] = True
            else:
                self.batt[1] = None
                self.cmd_com[0] = False
                if self.batt[0] >= 25.00:
                    set_single_input_register(slave_map_io['map_io']['status']['battery_indicator'], 0)
                elif self.batt[0] < 25.00 and self.batt[0] > config['Low_Voltage']:
                    set_single_input_register(slave_map_io['map_io']['status']['battery_indicator'], 1)
            # ======================================
            # ================= STATUS ======================================
            # =======================================================
            set_single_discrete_input(slave_map_io['map_io']['dashboard']['state']['running'], True)
            set_single_discrete_input(slave_map_io['map_io']['dashboard']['state']['out_of_track'], not (self.canbus.flag >> 1) & 1)
            set_single_discrete_input(slave_map_io['map_io']['dashboard']['alarm']['estop_area'], self.cmd_com[3])
            set_single_discrete_input(slave_map_io['map_io']['dashboard']['alarm']['alarm_fault'], self.cmd_com[0])
            set_single_input_register(slave_map_io['map_io']['dashboard']['rfid'], self.canbus.data_RFID)
            set_multiple_input_registers(slave_map_io['map_io']['dashboard']['clock']['hour'], get_uptime())         
            # ======================================
            # ================= SET DEFAULT SPEED ======================================
            # =======================================================
            if get_single_coil(slave_map_io['map_io']['dashboard']['control']['set_default_speed']):
                config['Speed'] = get_single_holding_register(slave_map_io['map_io']['dashboard']['speed'])
                save_config()
                set_single_coil(slave_map_io['map_io']['dashboard']['control']['set_default_speed'], False)
            # ======================================
            # ================= CALIBRATION MAGNET ======================================
            # =======================================================
            set_multiple_input_registers(slave_map_io['map_io']['setting']['sensor']['magnet']['left'], [int16_to_registers(self.canbus.sensor[0]), int16_to_registers(self.canbus.sensor[2])])
            if get_single_coil(slave_map_io['map_io']['setting']['sensor']['magnet']['calibration']):
                self.canbus.calibrate_sensor_magnet()
                set_single_coil(slave_map_io['map_io']['setting']['sensor']['magnet']['calibration'], False)
            # ======================================
            # ================= CALIBRATION RFID ======================================
            # =======================================================
            set_single_holding_register(slave_map_io['map_io']['setting']['sensor']['rfid']['actual_rfid'], self.canbus.data_RFID)
            if get_single_coil(slave_map_io['map_io']['setting']['sensor']['rfid']['calibration']):
                set_RFID(get_single_holding_register(slave_map_io['map_io']['setting']['sensor']['rfid']['set_rfid']))
                set_single_coil(slave_map_io['map_io']['setting']['sensor']['rfid']['calibration'], False)
            # ======================================
            # ================= CONFIGURATION ======================================
            # =======================================================
            if get_single_coil(slave_map_io['map_io']['setting']['save']):
                save_config()
                set_single_coil(slave_map_io['map_io']['setting']['save'], False)
            # ======================================
            # ================= MAIN ======================================
            # =======================================================
            if self.cmd_com[0]:
                self.cmd_com[1] = False
                self.canbus.set_kecepatan_motor([0, 0])
                self.lmp_out = [1, 0, 0]
            else:
                if not self.ssa__in[0] or self.ssa__in[4]:
                    self.commnad = False
                    self.canbus.set_kecepatan_motor([0, 0])
                    self.lmp_out = [1, 0, 0]
                else:
                    if self.cmd_com[1]:
                        # ======================================
                        # ================= BUMPER ======================================
                        # =======================================================
                        if self.ssa__in[3] == True:
                            self.cmd_com[3] = True
                        # ======================================
                        # =======================================================
                        # =======================================================
                        if not (self.canbus.flag >> 1) & 1:
                            self.canbus.set_kecepatan_motor([0, 0])
                            self.snd_out = [0, 1, 0, 0]
                            self.lmp_out = [1, 0, 0]
                        elif self.cmd_com[3]:
                            self.canbus.set_kecepatan_motor([0, 0])
                            self.snd_out = [0, 1, 0, 0]
                            self.lmp_out = [1, 0, 0]
                        else:
                            if self.tmp_com[2] != get_single_holding_register(slave_map_io['map_io']['dashboard']['speed']):
                                self.cmd_com[2] = False
                                self.tmp_com[2] = get_single_holding_register(slave_map_io['map_io']['dashboard']['speed'])

                            if self.cmd_com[2] == False:
                                self.previous_time = time.time() * 1000
                                self.cmd_com[2] = True
                            
                            self.select_track(get_single_holding_register(slave_map_io['map_io']['setting']['track']))
                            set_single_discrete_input(slave_map_io['map_io']['dashboard']['state']['forward'], 1)
                            if get_single_holding_register(slave_map_io['map_io']['setting']['obstacle']) != 0:
                                if self.ssa__in[2]:
                                    if not self.ssa__in[1]:
                                        self.snd_out = [1, 0, 0, 0]
                                        self.lmp_out = [0, 0, 1]
                                        set_multiple_discrete_inputs(slave_map_io['map_io']['dashboard']['alarm']['slow_area'], [1, 0, 0, 0])
                                        target_speed = [int(speed(20) + self.pid_slow.compute(self.error)), int(-speed(20) + self.pid_slow.compute(self.error))]
                                        self.canbus.set_kecepatan_motor(self.update_speed_slow(target_speed))
                                        self.cmd_com[2] = False
                                    else:
                                        if not get_single_holding_register(slave_map_io['map_io']['setting']['music']):
                                            self.snd_out = [0, 0, 0, 0]
                                        else:
                                            self.snd_out = [0, 0, 0, 1]
                                        self.lmp_out = [0, 1, 0]
                                        set_multiple_discrete_inputs(slave_map_io['map_io']['dashboard']['alarm']['slow_area'], [0, 0, 0, 0])
                                        target_speed = [int(speed(get_single_holding_register(slave_map_io['map_io']['dashboard']['speed'])) + self.pid.compute(self.error)), int(-speed(get_single_holding_register(slave_map_io['map_io']['dashboard']['speed'])) + self.pid.compute(self.error))]
                                        if time.time() * 1000 - self.previous_time <= self.interval:
                                            self.canbus.set_kecepatan_motor(self.update_speed(target_speed))
                                        else:
                                            self.ss__spd = target_speed
                                            self.canbus.set_kecepatan_motor(target_speed)
                                else:
                                    self.snd_out = [0, 1, 0, 0]
                                    self.lmp_out = [1, 0, 0]
                                    set_multiple_discrete_inputs(slave_map_io['map_io']['dashboard']['alarm']['slow_area'], [0, 1, 0, 0])
                                    self.canbus.set_kecepatan_motor([int(self.pid.compute(self.error)), int(self.pid.compute(self.error))])
                                    self.ss__spd = [0, 0]
                                    self.cmd_com[2] = False
                            else:
                                if not get_single_holding_register(slave_map_io['map_io']['setting']['music']):
                                    self.snd_out = [0, 0, 0, 0]
                                else:
                                    self.snd_out = [0, 0, 0, 1]
                                self.lmp_out = [0, 1, 0]
                                set_multiple_discrete_inputs(slave_map_io['map_io']['dashboard']['alarm']['slow_area'], [0, 0, 0, 0])
                                target_speed = [int(speed(get_single_holding_register(slave_map_io['map_io']['dashboard']['speed'])) + self.pid.compute(self.error)), int(-speed(get_single_holding_register(slave_map_io['map_io']['dashboard']['speed'])) + self.pid.compute(self.error))]
                                if time.time() * 1000 - self.previous_time <= self.interval:
                                    self.canbus.set_kecepatan_motor(self.update_speed(target_speed))
                                else:
                                    self.ss__spd = target_speed
                                    self.canbus.set_kecepatan_motor(target_speed) 
                    else:
                        self.snd_out = [0, 0, 0, 0]
                        self.lmp_out = [0, 0, 0]
                        set_multiple_discrete_inputs(slave_map_io['map_io']['dashboard']['alarm']['slow_area'], [0, 0, 0, 0])
                        if self.cmd_com[2] == False:
                            self.previous_time = time.time() * 1000
                            self.cmd_com[2] = True
                        set_single_discrete_input(slave_map_io['map_io']['dashboard']['state']['forward'], 0)
                        target_speed = [0, 0]
                        if time.time() * 1000 - self.previous_time <= self.interval:
                            self.canbus.set_kecepatan_motor(self.update_speed(target_speed))
                        else:
                            self.ss__spd = target_speed
                            self.canbus.set_kecepatan_motor(target_speed)
            data = [
                get_cpu_temperature(), 
                self.canbus.temp_driver, 
                self.canbus.error[1], 
                self.canbus.error[0], 
                self.canbus.current[1], 
                self.canbus.current[0], 
                self.cc___er[0], 
                self.canbus.can_open, 
                self.cmd_com[3],
                self.canbus.voltage - 0.22
            ]
            status(data, com=self.cc___er[1])


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
