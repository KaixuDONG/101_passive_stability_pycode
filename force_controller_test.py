"""
this is the code to test the thrust controller by the help of logging the thrust in Newton
from the robot as feedback

-> June 3, 2021


-> Nov 1, 2021
------!!!!-----------
now we have a new version of force controller which is online, flashed in the firmware 
the code is of less usage
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

def limit_thrust(thrust_cmd):
    if thrust_cmd > 65535:
        thrust_cmd = 65535
    if thrust_cmd < 0:
        thrust_cmd = 0
    return int(thrust_cmd)


# set the sample rate
sample_rate = 100.0
sample_time = 1.0 / sample_rate
sample_rate_in_ms = int(sample_time * 1000)


take_off_flag = False
land_flag = False
flight_time = 0.0

thrust_base = 5000
thrust_bas_take_off = 25000
thrust_cmd_I = 29000

thrust_count = 0
thrust_slope = 100

thrust_inc_flag = False
thrust_dec_flag = False

flight_time_index = int(1)

estimated_height = 0
estimated_veloccity = 0

# the desired thrust set, in Newton
desired_thrust = 2.85
# thrust_p_gain = 200

# thrust_I_gain = 80
# thrust_P_gain = 10
# feedforward_term = 1000

thrust_I_gain = 60
thrust_P_gain = 30
feedforward_term = 8000

thrust_err_limit = 0.15

desired_roll = -8.6
# desired_thrust_payload = 2.56



# set the selected crazyflie's URI
URI = 'radio://0/101/2M'

# Only output errors from tinthe logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('externalforce.T', 'float')

    saver_follower_1 = savemat.DataSaver('roll_actual', 'pitch_actual', 'thrust_actual')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        scf.cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)

        # scf.cf.param.set_value('kalman.resetEstimation', '0')
        # time.sleep(0.1)

        with SyncLogger(scf, lg_stab) as logger:
            # main loop here
            for log_entry in logger:
                joystick.step()

                thrust_inc_flag_prev = thrust_inc_flag
                thrust_dec_flag_prev = thrust_dec_flag
                
                # actually the rising edge detection
                # rb -> increase the thrust
                if joystick.get_button_rb():
                    thrust_inc_flag = True
                    if (not thrust_inc_flag_prev) and  thrust_inc_flag:
                        thrust_count = int(thrust_count + 1)
                else:
                    thrust_inc_flag = False 

                # lb -> decrease the thrust
                if joystick.get_button_lb():
                    thrust_dec_flag = True
                    if (not thrust_dec_flag_prev) and  thrust_dec_flag:
                        thrust_count = int(thrust_count - 1)
                else:
                    thrust_dec_flag = False

                # button_l -> take off
                if joystick.get_button_l():
                    take_off_flag = True

                # push the axis lt and rt together to initiate the autonomous landing (vertically)
                if joystick.get_axis_lt()>0.95 and joystick.get_axis_rt()>0.95:
                    land_flag = True

                # staying on the ground with propeller spining
                if not take_off_flag:
                    # print('robot staying on the ground')
                    scf.cf.commander.send_setpoint(0, 0, 0, thrust_base)
                    time.sleep(0.01)
                    # terminate the flight procedure
                    if joystick.get_button_x():
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break

                # in air flight
                if take_off_flag and not land_flag:
                    # scf.cf.commander.send_setpoint(0, 0, 0, thrust_bas_take_off + thrust_count*thrust_slope)
                    thrust_error_inNewton = desired_thrust - log_entry[1].get('externalforce.T')
                    if thrust_error_inNewton >= thrust_err_limit:
                        thrust_error_inNewton = thrust_err_limit
                    if thrust_error_inNewton <= -thrust_err_limit:
                        thrust_error_inNewton = -thrust_err_limit

                    thrust_cmd_I = int(thrust_cmd_I + thrust_I_gain*thrust_error_inNewton)

                    thrust_cmd = thrust_cmd_I + thrust_error_inNewton*thrust_P_gain + feedforward_term

                    thrust_cmd = limit_thrust(thrust_cmd)

                    print(thrust_cmd)
                    
                    scf.cf.commander.send_setpoint(0, 0, 0, thrust_cmd)

                    saver_follower_1.add_elements(log_entry[1].get('stabilizer.roll'), 
                            log_entry[1].get('stabilizer.pitch'),
                            log_entry[1].get('externalforce.T'))

                    flight_time_index = flight_time_index + 1
                    if flight_time_index <= 3000:
                        desired_thrust = 2.85
                    elif  flight_time_index > 3000 and flight_time_index <= 6000:
                        desired_thrust = 2.15
                    else:
                        desired_thrust = 2.55
                    
                    # terminate the flight in emergency
                    if joystick.get_button_x():
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break
                
                # land the robot
                if joystick.get_button_x():
                    scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                    time.sleep(0.2)
                    scf.cf.commander.send_setpoint(0, 0, 0, 0)
                    break
    
    saver_follower_1.save2mat('DATA_thrust_control_test/')
    joystick.quit()