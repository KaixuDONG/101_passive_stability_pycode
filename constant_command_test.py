"""
this is the code to test a constant commmand
meaning sending constant thrust cmd and attitude commmands to the robot

caution, attach a cable to the ground
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

from rislab_lib.Joystick import joystick

# set the sample rate
sample_rate = 100.0
sample_time = 1.0 / sample_rate
sample_rate_in_ms = int(sample_time * 1000)


# default values of  the desired states
desired_vx = 0.0
desired_vy = 0.0
desired_yaw_rate = 0.0
desired_height = 0.2

desired_x_pos = 0.0
desired_y_pos = 0.0
desired_z_pos = 0.2

take_off_flag = False
flight_time = 0.0

# set the selected crazyflie's URI
URI = 'radio://0/23/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
    # lg_stab.add_variable('stabilizer.roll', 'float')
    # lg_stab.add_variable('stabilizer.pitch', 'float')
    # lg_stab.add_variable('stabilizer.yaw', 'float')
    lg_stab.add_variable('acc.z', 'float')
    lg_stab.add_variable('stabilizer.thrust', 'float')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        scf.cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)

        # scf.cf.param.set_value('kalman.resetEstimation', '0')
        # time.sleep(0.1)

        with SyncLogger(scf, lg_stab) as logger:
            for log_entry in logger:
                joystick.step()

                if not take_off_flag:
                    # print('robot staying on the ground')
                    scf.cf.commander.send_setpoint(0, 0, 0, 5000)
                    time.sleep(0.1)

                # for y in range(10):
                #     scf.cf.commander.send_hover_setpoint(0, 0, 0, y / 25)
                #     time.sleep(0.1)

                # for _ in range(30):
                #     scf.cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
                #     time.sleep(0.1)

                if joystick.get_button_l():
                    take_off_flag = True


                if take_off_flag:

                    # fly upwards for 1 sec
                    if flight_time < 0.8:
                        scf.cf.commander.send_setpoint(0, 0, 0, 44000)  
                        time.sleep(0.1)
                    else: 
                        scf.cf.commander.send_setpoint(1.5, 1.5, 0, 52000)  
                        time.sleep(0.1)

                    # scf.cf.commander.send_setpoint(0.5, 0, 0, 50000)  
                    # time.sleep(0.1)

                    # if joystick.get_button_y():
                    #     scf.cf.commander.send_setpoint(0, 0, 0, 50000)

                    # elif joystick.get_button_a():
                    #     scf.cf.commander.send_setpoint(-3, 0, 0, 50000)
                    
                    # elif joystick.get_button_b():
                    #     scf.cf.commander.send_setpoint(0, 3, 0, 50000)

                    # else:
                    #     desired_vx = -joystick.get_axis_ly() * 0.5
                    #     desired_vy = -joystick.get_axis_lx() * 0.5
                    #     desired_yaw_rate = joystick.get_axis_rx() * 30

                    #     desired_height = desired_height - joystick.get_axis_ry() * sample_time * 0.4
                    #     scf.cf.commander.send_hover_setpoint(desired_vx, desired_vy, desired_yaw_rate, desired_height)

                    # desired_x_pos = desired_x_pos - joystick.get_axis_ly() * sample_time * 0.5
                    # desired_y_pos = desired_y_pos - joystick.get_axis_lx() * sample_time * 0.5
                    # desired_yaw_rate = joystick.get_axis_rx() * 90

                    # desired_z_pos = desired_z_pos - joystick.get_axis_ry() * sample_time * 0.4
                    # scf.cf.commander.send_position_setpoint(desired_x_pos, desired_y_pos, desired_z_pos, desired_yaw_rate)
                   
                    flight_time += sample_time
                    print(flight_time)

                    if joystick.get_button_x():
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)

                        break

    joystick.quit()