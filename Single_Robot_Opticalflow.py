"""
this is the  program for  a single crazyflie using optical flow to fly

checked in NOV 1, 2021, suitable for MacOS
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

from rislab_lib.Joystick import joystick
from cooperative_transport.controllers import Creat_Agent

# set the sample rate
sample_rate = 100.0
sample_time = 1.0 / sample_rate
sample_rate_in_ms = int(sample_time * 1000)

# default values of  the desired states
# creat the crazyflie agent
leader = Creat_Agent()

leader.pos_z.des = 0.2

constant_velocity = 0.2

take_off_flag = False
constant_velocity_away_flag = False
constant_velocity_back_flag = False
controlled_by_pilot_flag = True


# set the selected crazyflie's URI
URI = 'radio://0/31/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
    lg_stab.add_variable('acc.z', 'float')
    lg_stab.add_variable('stabilizer.thrust', 'float')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.set_value('stabilizer.controller', '1')
        scf.cf.param.set_value('stabilizer.estimator', '2')

        scf.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        scf.cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

        scf.cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)

        with SyncLogger(scf, lg_stab) as logger:
            for log_entry in logger:
                joystick.step()

                if joystick.get_button_l():
                    take_off_flag = True

                if joystick.get_button_a():
                    controlled_by_pilot_flag = False
                    constant_velocity_away_flag = True
                    constant_velocity_back_flag = False
                
                if joystick.get_button_b():
                    controlled_by_pilot_flag = False
                    constant_velocity_away_flag = False
                    constant_velocity_back_flag = True

                if joystick.get_button_y():
                    controlled_by_pilot_flag = True
                    constant_velocity_away_flag = False
                    constant_velocity_back_flag = False

                if not take_off_flag:
                    scf.cf.commander.send_setpoint(0, 0, 0, 5000)
                    time.sleep(0.1)

                if take_off_flag:
                    if controlled_by_pilot_flag:
                        leader.vel_x.des = -joystick.get_axis_ly() * 0.5
                        leader.vel_y.des = -joystick.get_axis_lx() * 0.5
                        leader.yaw_rate.des = joystick.get_axis_rx() * 30
                        leader.pos_z.des =  leader.pos_z.des - joystick.get_axis_ry() * sample_time * 0.4

                    # if constant_velocity_away_flag:
                    #     desired_vx = 0.0
                    #     desired_vy = constant_velocity
                    #     desired_yaw_rate = 0.0

                    # if constant_velocity_away_flag:
                    #     desired_vx = 0.0
                    #     desired_vy = -constant_velocity
                    #     desired_yaw_rate = 0.0
                    
                    scf.cf.commander.send_hover_setpoint(leader.vel_x.des, leader.vel_y.des, leader.yaw_rate.des, leader.pos_z.des)

                    # desired_x_pos = desired_x_pos - joystick.get_axis_ly() * sample_time * 0.5
                    # desired_y_pos = desired_y_pos - joystick.get_axis_lx() * sample_time * 0.5
                    # desired_yaw_rate = joystick.get_axis_rx() * 90

                    # desired_z_pos = desired_z_pos - joystick.get_axis_ry() * sample_time * 0.4
                    # scf.cf.commander.send_position_setpoint(desired_x_pos, desired_y_pos, desired_z_pos, desired_yaw_rate)

                    if joystick.get_button_x():
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)

                        break

    joystick.quit()
