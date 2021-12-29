"""
this is the programme to test the onboard force controller of the followers

!!----
NOV 1, 2021
the buttons of the joystick are remapped 
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

from rislab_lib.Joystick import joystick
from rislab_lib.savemat import savemat


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

constant_velocity = 0.2

take_off_flag = False
land_flag = False
# constant_velocity_away_flag = False
# constant_velocity_back_flag = False
# controlled_by_pilot_flag = True


# set the selected crazyflie's URI
URI = 'radio://0/66/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

axis_act_thre = 0.95
axis_deact_thre = 0.05

take_off_time = 0.0
land_time = 0.0

# since there are 4 motors
coe_newton_to_cmd = 10000/4

desired_thrust_inNewton = 2.6496

desired_roll = -3
desired_pitch = -4.5


saver_follower_1 = savemat.DataSaver('modeled_thrust', 
                                     'roll', 
                                     'pitch', 
                                     'roll_rate', 
                                     'pitch_rate')

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
    
    lg_stab.add_variable('externalforce.T', 'float')
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('gyro.x', 'float')
    lg_stab.add_variable('gyro.y', 'float')


    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.set_value('stabilizer.controller', '1')
        # scf.cf.param.set_value('stabilizer.estimator', '1')

        # scf.cf.param.set_value('kalman.resetEstimation', '1')
        # time.sleep(0.1)
        # scf.cf.param.set_value('kalman.resetEstimation', '0')
        # time.sleep(2)

        scf.cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)

        # scf.cf.param.set_value('kalman.resetEstimation', '0')
        # time.sleep(0.1)

        with SyncLogger(scf, lg_stab) as logger:
            for log_entry in logger:
                
                force_modeled = log_entry[1].get('externalforce.T')

                joystick.step()

                if joystick.get_button_l():
                    take_off_flag = True
                    take_off_time = time.time()

                if joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() > axis_act_thre:
                    land_flag = True
                    land_time = time.time()

                # if joystick.get_button_a():
                #     controlled_by_pilot_flag = False
                #     constant_velocity_away_flag = True
                #     constant_velocity_back_flag = False
                
                # if joystick.get_button_b():
                #     controlled_by_pilot_flag = False
                #     constant_velocity_away_flag = False
                #     constant_velocity_back_flag = True

                # if joystick.get_button_y():
                #     controlled_by_pilot_flag = True
                #     constant_velocity_away_flag = False
                #     constant_velocity_back_flag = False

                if not take_off_flag:
                    # print('robot staying on the ground')
                    scf.cf.commander.send_setpoint(0, 0, 0, 600)
                    time.sleep(0.1)
                    
                    if joystick.get_button_x():
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break

                if take_off_flag and not land_flag:
                    if (time.time() - take_off_time) <= 6:
                        time_scale = (time.time() - take_off_time)/6
                        desired_thrust_inNewton_temp = time_scale*2.4
                        desired_roll_temp = time_scale*0
                        desired_pitch_temp = time_scale*0
                        scf.cf.commander.send_setpoint(desired_roll_temp, desired_pitch_temp, 0, int(desired_thrust_inNewton_temp * coe_newton_to_cmd))
                    elif (time.time() - take_off_time) <= 9:
                        scf.cf.commander.send_setpoint(-6, desired_pitch*0.7, 0, int(2.5 * coe_newton_to_cmd))
                    elif (time.time() - take_off_time) <= 12:
                        scf.cf.commander.send_setpoint(-7, desired_pitch*0.9, 0, int(2.7 * coe_newton_to_cmd))
                    else:
                        scf.cf.commander.send_setpoint(desired_roll, desired_pitch, 0, int(desired_thrust_inNewton * coe_newton_to_cmd))

                    saver_follower_1.add_elements(force_modeled, log_entry[1].get('stabilizer.roll'),
                    log_entry[1].get('stabilizer.pitch'),
                    log_entry[1].get('gyro.x'),
                    log_entry[1].get('gyro.y'),
                    )
                    
                    # print(force_modeled)

                    if joystick.get_button_x():
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break

                if land_flag:
                    if (time.time() - land_time) <= 7:
                        time_scale = 1 - (time.time() - land_time)/7

                        if time_scale <= 0.05:
                            time_scale = 0.05

                        desired_thrust_inNewton_temp = time_scale*desired_thrust_inNewton
                        desired_roll_temp = time_scale*desired_roll
                        desired_pitch_temp = time_scale*desired_pitch

                        scf.cf.commander.send_setpoint(desired_roll_temp, 0, 0, int(desired_thrust_inNewton_temp * coe_newton_to_cmd))
                    else:
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)

                    if joystick.get_button_x():
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break

    saver_follower_1.save2mat('DATA_Thrust_Onboard_Control/')
    joystick.quit()