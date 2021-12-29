"""
this is the program using optical flow deck of crazy flie to control a single quadrotor 
by a joystick

-> version 1.0, May 5, 2021
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


vel_x_des_leader = 0.0
vel_y_des_leader = 0.0
yaw_rate_des_leader = 0.0
height_des_leader = 0.2
# the max velocity set
max_vel = 0.3

# controller parameters for the leader
height_P_gain = 15000
height_I_gain = 5000
height_D_gain = 8000

# height_P_gain = 9000
# height_I_gain = 5000    
# height_I_gain = 5000

height_D_gain = 0
    
vel_x_P_gain = 10
vel_x_I_gain = 0.5

vel_y_P_gain = 10
vel_y_I_gain = 0.5

height_err_inte = 0.0
vel_x_err_inte = 0.0
vel_y_err_inte = 0.0

height_err_inte_max = 1000
vel_err_inte_max = 300

thrust_feedforward_leader = 30000

thrust_cmd_leader_max = 50000
pitch_cmd_leader_max = 10 # in degree
roll_cmd_leader_max = 10 # in degree

constant_velocity = 0.2

take_off_flag = False
constant_velocity_away_flag = False
constant_velocity_away_flag = False

constant_velocity_back_flag = False
controlled_by_pilot_flag = True

axis_act_thre = 0.97
# controller parameters set

# set the selected crazyflie's URI
URI = 'radio://0/31/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

height_estimated = EKF_Estimator(sample_time)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    lg_stab_leader = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
    lg_stab_leader.add_variable('acc.z', 'float')
    lg_stab_leader.add_variable('acc.y', 'float')
    lg_stab_leader.add_variable('stateEstimate.roll', 'float')
    lg_stab_leader.add_variable('stateEstimateZ.z', 'int16_t')
    lg_stab_leader.add_variable('stateEstimateZ.vz', 'int16_t')
    lg_stab_leader.add_variable('stateEstimateZ.vx', 'int16_t')
    lg_stab_leader.add_variable('stateEstimateZ.vy', 'int16_t')


    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        scf.cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

        scf.cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)

        # scf.cf.param.set_value('kalman.resetEstimation', '0')
        # time.sleep(0.1)

        with SyncLogger(scf, lg_stab_leader) as logger:
            for log_entry in logger:
                joystick.step()

                # log and process the data
                data_leader = log_entry[1]

                if joystick.get_button_l():
                    take_off_flag = True


                if not take_off_flag:
                    scf.cf.commander.send_setpoint(0, 0, 0, 5000)
                    time.sleep(0.1)

                if take_off_flag:

                    # update the feedback and estimation
                    az_imu = (data_leader['acc.y']*9.81*math.sin(math.radians(data_leader['stateEstimate.roll'])) + data_leader['acc.z']*9.81*math.cos(math.radians(data_leader['stateEstimate.roll'])) - 9.81)
                    height_fdK_leader, vel_z_fdk_leader = height_estimated.EstimatorZ(az_imu,float(data_leader['stateEstimateZ.vz'])/1000,float(data_leader['stateEstimateZ.z'])/1000)
                    
                    # update the desired value of velocity x and y and height, commands given by joystick
                    # if joystick.get_axis_lt() > axis_act_thre and joystick.get_axis_rt() < -axis_act_thre:
                    #     vel_x_des_leader = -joystick.get_axis_ly() * max_vel
                    #     vel_y_des_leader = -joystick.get_axis_lx() * max_vel
                    #     yaw_rate_des_leader = joystick.get_axis_rx() * 30
                    #     height_des_leader = height_des_leader - joystick.get_axis_ry() * sample_time * 0.4
                    # else:
                    #     vel_x_des_leader = 0
                    #     vel_y_des_leader = 0
                    vel_x_des_leader = -joystick.get_axis_ly() * max_vel
                    vel_y_des_leader = -joystick.get_axis_lx() * max_vel
                    yaw_rate_des_leader = joystick.get_axis_rx() * 30
                    height_des_leader = height_des_leader - joystick.get_axis_ry() * sample_time * 0.4

                    # update the error
                    height_err_inB = height_des_leader - height_fdK_leader
                    vel_z_err_inB = 0 - vel_z_fdk_leader

                    vel_x_err_inB = vel_x_des_leader - float(data_leader['stateEstimateZ.vx'])/1000
                    vel_y_err_inB = vel_y_des_leader - float(data_leader['stateEstimateZ.vy'])/1000

                    # update the integrator
                    height_err_inte = height_err_inte + height_err_inB*sample_time
                    height_err_inte = saturation_function(height_err_inte, -height_err_inte_max, height_err_inte_max)

                    vel_x_err_inte = vel_x_err_inte + vel_x_err_inB*sample_time
                    vel_x_err_inte = saturation_function(vel_x_err_inte, -vel_err_inte_max, vel_err_inte_max)

                    vel_y_err_inte = vel_y_err_inte + vel_y_err_inB*sample_time
                    vel_y_err_inte = saturation_function(vel_y_err_inte, -vel_err_inte_max, vel_err_inte_max)

                    # update the controller output
                    thrust_cmd_leader = height_P_gain*height_err_inB + height_D_gain*vel_z_err_inB + height_I_gain*height_err_inte + thrust_feedforward_leader
                    thrust_cmd_leader = int(saturation_function(thrust_cmd_leader, 0, thrust_cmd_leader_max))

                    pitch_cmd_leader = vel_x_P_gain*vel_x_err_inB + vel_x_I_gain*vel_x_err_inte
                    pitch_cmd_leader = saturation_function(pitch_cmd_leader, -pitch_cmd_leader_max, pitch_cmd_leader_max)

                    roll_cmd_leader = -(vel_y_P_gain*vel_y_err_inB + vel_y_I_gain*vel_y_err_inte)
                    roll_cmd_leader = saturation_function(roll_cmd_leader, -roll_cmd_leader_max, roll_cmd_leader_max)

                    # print('height_fdK_leader=', height_fdK_leader, '    ',
                    # 'height_des_leader = ', height_des_leader, '  ',
                    # 'height_err_inB = ', height_err_inB, '  ',
                    # 'thrust_cmd_leader = ', thrust_cmd_leader)

                    # scf.cf.commander.send_hover_setpoint(vel_x_des, vel_y_des, yaw_rate_des, pos_z_des)
                    # scf.cf.commander.send_setpoint(roll_cmd_leader, pitch_cmd_leader, 0.0, thrust_cmd_leader)
                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, thrust_cmd_leader)

                    if joystick.get_button_x():
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break

    joystick.quit()
