"""
this is the programe for testing the logic of autonomous taking off and landing, for the follower

this program does not need mocap

***the mapping between the thrust cmd and the actual thrust force will be added 

-> June 3, 2021


-> Nov 1, 2021
----------!!!!------
the thrust control for the followers uses the logged data from onboard, which is not precise and 
with time delay

the new version of the code uses the onboard thrust control and the 
thrust commmand is directly given by 'sendsetpoint'

this code is an old version and should not be used again
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


take_off_flag = False
land_flag = False
flight_time = 0.0

thrust_base = 5000
thrust_bas_take_off = 25000
thrust_cmd = thrust_bas_take_off

thrust_count = 0
thrust_slope = 100

thrust_inc_flag = False
thrust_dec_flag = False

flight_time_index = int(1)

estimated_height = 0
estimated_veloccity = 0

# the desired thrust set, in Newton
desired_thrust = 2.87
# thrust_p_gain = 200
thrust_p_gain = 80

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
    # lg_stab.add_variable('stabilizer.roll', 'float')
    # lg_stab.add_variable('stabilizer.pitch', 'float')
    # lg_stab.add_variable('stabilizer.yaw', 'float')
    lg_stab.add_variable('acc.z', 'float')
    lg_stab.add_variable('stabilizer.thrust', 'float')
    lg_stab.add_variable('externalforce.T', 'float')


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
                    thrust_cmd = int(thrust_cmd + thrust_p_gain*thrust_error_inNewton)
                    
                    if flight_time_index <= 110:
                        scf.cf.commander.send_setpoint(-6.5, 0, 0, 33000)
                    else:
                        scf.cf.commander.send_setpoint(desired_roll, -0.3, 0, thrust_cmd)


                    # scf.cf.commander.send_setpoint(0, 0, 0, thrust_bas_take_off + thrust_count*thrust_slope)

                    # to estimate the height based on the acc_z, to decide when to finish the vertical taking off
                    # and start to roll, but the estimation is not that precise 
                    acc_z = (log_entry[1].get('acc.z') - 1)*9.81
                    estimated_height = estimated_height + estimated_veloccity*sample_time + 0.5*acc_z*sample_time**2
                    estimated_veloccity = acc_z*sample_time

                    flight_time_index += 1
                    if flight_time_index % 50 == 0:
                        print('thrust_cmd = ', log_entry[1].get('stabilizer.thrust'),
                               ' ;acc_z = ', acc_z,
                               ' ;thrust_inNewton = ', log_entry[1].get('externalforce.T'),
                            #    ' ;estimated_height = ', estimated_height
                               )    
                    
                    # terminate the flight in emergency
                    if joystick.get_button_x():
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break
                
                # land the robot
                if land_flag:
                    flight_time_index += 1
                    if flight_time_index % 50 == 0:
                        print('robot is landing')
                    
                    scf.cf.commander.send_setpoint(-5, 0, 0, 32000)
                    # terminate this whole flight procedure when the robot is landed
                    if joystick.get_button_x():
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break

    joystick.quit()