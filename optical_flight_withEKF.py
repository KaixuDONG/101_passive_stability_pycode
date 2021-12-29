"""
this is the program using optical flow deck of crazy flie to control a single quadrotor 
by a joystick

-> version 1.0, May 5, 2021

this is the version that uses a EKF for optical flow, 
to solve the problem 
that the height estimation is not precise

but now the kalman filter for the height estimation is implemented in the firmware,
so the code is of very limited use
-----!!!!-------
Nov 1, 2021

"""
import logging
import time
import math

import cflib.crtp
from cflib.crazyflie import Crazyflie

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

from rislab_lib.Joystick import joystick

from cooperative_transport.controllers import EKF_Estimator
from cooperative_transport.controllers import saturation_function 




# set the sample rate
sample_rate = 100.0
sample_time = 1.0 / sample_rate
sample_rate_in_ms = int(sample_time * 1000)


# default values of  the desired states
vel_x_des = 0.0
vel_y_des = 0.0
yaw_rate_des = 0.0
pos_z_des = 0.2

desired_x_pos = 0.0
desired_y_pos = 0.0
desired_z_pos = 0.2

constant_velocity = 0.2

take_off_flag = False
constant_velocity_away_flag = False
constant_velocity_away_flag = False


constant_velocity_back_flag = False
controlled_by_pilot_flag = True

# controller parameters set
pos_z_P_gain = 15000
pos_z_I_gain = 5000
pos_z_D_gain = 8000

vel_x_P_gain = 10
vel_x_I_gain = 0.5

vel_y_P_gain = 10
vel_y_I_gain = 0.5

pos_z_err_inte = 0.0
vel_x_err_inte = 0.0
vel_y_err_inte = 0.0

thrust_feedforward = 31000

thrust_cmd_max = 45000
pitch_cmd_max = 10
roll_cmd_max = 10

# set the selected crazyflie's URI
URI = 'radio://0/31/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

height_estimated = EKF_Estimator(sample_time)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
    lg_stab.add_variable('acc.z', 'float')
    lg_stab.add_variable('acc.y', 'float')
    lg_stab.add_variable('stateEstimate.roll', 'float')
    lg_stab.add_variable('stateEstimateZ.z', 'int16_t')
    lg_stab.add_variable('stateEstimateZ.vz', 'int16_t')
    lg_stab.add_variable('stateEstimateZ.vx', 'int16_t')
    lg_stab.add_variable('stateEstimateZ.vy', 'int16_t')


    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        scf.cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

        scf.cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)

        # scf.cf.param.set_value('kalman.resetEstimation', '0')
        # time.sleep(0.1)

        with SyncLogger(scf, lg_stab) as logger:
            for log_entry in logger:
                joystick.step()

                # log and process the data
                data = log_entry[1]

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

                        # update the feedback and estimation
                        az_imu = (data['acc.y']*9.81*math.sin(math.radians(data['stateEstimate.roll'])) + data['acc.z']*9.81*math.cos(math.radians(data['stateEstimate.roll'])) - 9.81)
                        pos_z_fdk, vel_z_fdk = height_estimated.EstimatorZ(az_imu,float(data['stateEstimateZ.vz'])/1000,float(data['stateEstimateZ.z'])/1000)

                        # the desired states
                        vel_x_des = -joystick.get_axis_ly() * 0.5
                        vel_y_des = -joystick.get_axis_lx() * 0.5
                        yaw_rate_des = joystick.get_axis_rx() * 30
                        pos_z_des = pos_z_des - joystick.get_axis_ry() * sample_time * 0.3

                        # update the error
                        pos_z_err_inB = pos_z_des - pos_z_fdk
                        vel_z_err_inB = 0 - vel_z_fdk

                        vel_x_err_inB = vel_x_des - float(data['stateEstimateZ.vx'])/1000
                        vel_y_err_inB = vel_y_des - float(data['stateEstimateZ.vy'])/1000

                        # update the integrator
                        pos_z_err_inte = pos_z_err_inte + pos_z_err_inB*sample_time
                        pos_z_err_inte = saturation_function(pos_z_err_inte, -1000, 1000)

                        vel_x_err_inte = vel_x_err_inte + vel_x_err_inB*sample_time
                        vel_x_err_inte = saturation_function(vel_x_err_inte, -300, 300)

                        vel_y_err_inte = vel_y_err_inte + vel_y_err_inB*sample_time
                        vel_y_err_inte = saturation_function(vel_y_err_inte, -300, 300)

                        # update the controller output
                        thrust_cmd = pos_z_P_gain*pos_z_err_inB + pos_z_D_gain*vel_z_err_inB + pos_z_I_gain*pos_z_err_inte + thrust_feedforward
                        thrust_cmd = int(saturation_function(thrust_cmd, 0, thrust_cmd_max))

                        pitch_cmd = vel_x_P_gain*vel_x_err_inB + vel_x_I_gain*vel_x_err_inte
                        pitch_cmd = saturation_function(pitch_cmd, -pitch_cmd_max, pitch_cmd_max)

                        # roll_cmd = vel_y_P_gain*vel_y_err_inB + vel_y_I_gain*vel_y_err_inte
                        roll_cmd = -(vel_y_P_gain*vel_y_err_inB + vel_y_I_gain*vel_y_err_inte)

                        roll_cmd = saturation_function(roll_cmd, -roll_cmd_max, roll_cmd_max)

                        # scf.cf.commander.send_hover_setpoint(vel_x_des, vel_y_des, yaw_rate_des, pos_z_des)
                        scf.cf.commander.send_setpoint(roll_cmd, pitch_cmd, 0.0, thrust_cmd)
                        # scf.cf.commander.send_setpoint(0, 0, 0.0, thrust_cmd)


                    if joystick.get_button_x():
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)

                        break

    joystick.quit()
