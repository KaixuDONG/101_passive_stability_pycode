
"""
this is the code for a formation with a leader and a follower
the leader is equipped with a optical flow but use EKF in the python
the follower is passive

but since the leader uses the logged data from the leader, 
there are severly delay and the delay accumaltes
resulting in unstable manner, especially when there are two robots

so the code is of less use
"""

import logging
import time
import math

# libs from crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.swarm import CachedCfFactory # -> for swarm
from cflib.crazyflie.swarm import Swarm # -> for swarm

# libs from rislab
from rislab_lib.mocap_udp import UdpReceiver
from rislab_lib.Joystick import joystick
from rislab_lib.savemat import savemat
from rislab_lib.controllers import pid_3d
from rislab_lib.data_processor import GeneralFcn
from rislab_lib.controllers import trajectory_loader
from cooperative_transport.controllers import EKF_Estimator
from cooperative_transport.controllers import saturation_function 

# libs from Estel
from cooperative_transport.controllers import PID_ControllerThreeAixs
from cooperative_transport.controllers import Sys_State_def
from cooperative_transport.wheel_functions import limit_angular

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
# saturation funciton for thrust
def limit_thrust(thrust_cmd):
    if thrust_cmd > 65535:
        thrust_cmd = 65535
    if thrust_cmd < 0:
        thrust_cmd = 0
    return int(thrust_cmd)

# crazyflie_control is the main control function for the swarm control which include the leader and follower 1
def crazyflie_control(scf,control_which_flag):
    """
    set the controller for this carzyflie in the firmware
    1 -> PID controller
    2 -> Mellinger controller
    3 -> INDI controller
    """
    # set the controller for all the crazyflies in the swarm to be PID 
    scf.cf.param.set_value('stabilizer.controller', '1')
    time.sleep(0.1)

    scf.cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    scf.cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

    # unlock all the motors, this step is very significant
    scf.cf.commander.send_setpoint(0, 0, 0, 0)
    time.sleep(0.1)

    axis_act_thre = 0.95
    axis_deact_thre = 0.05

    # ------- leader control branch -------
    if control_which_flag[0] == 0.0:
        lg_stab_leader = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
        lg_stab_leader.add_variable('acc.z', 'float')
        lg_stab_leader.add_variable('acc.y', 'float')
        lg_stab_leader.add_variable('stateEstimate.roll', 'float')
        lg_stab_leader.add_variable('stateEstimateZ.z', 'int16_t')
        lg_stab_leader.add_variable('stateEstimateZ.vz', 'int16_t')
        lg_stab_leader.add_variable('stateEstimateZ.vx', 'int16_t')
        lg_stab_leader.add_variable('stateEstimateZ.vy', 'int16_t')
        # lg_stab_leader.add_variable('stabilizer.int_thrust', 'uint16_t')
        vel_x = Sys_State_def()
        vel_y = Sys_State_def()
        vel_z = Sys_State_def()
        yaw_rate = Sys_State_def()
        pos_z = Sys_State_def()
        pos_z.des = 0.2

        # the max velocity set
        max_vel = 0.3

        pos_z_P_gain = 15000
        pos_z_I_gain = 5000
        pos_z_D_gain = 8000

        vel_x_P_gain = 10
        vel_x_I_gain = 0.5

        vel_y_P_gain = 10
        vel_y_I_gain = 0.5

        thrust_feedforward = 31000

        thrust_cmd_max = 45000
        pitch_cmd_max = 10
        roll_cmd_max = 10


        height_estimated = EKF_Estimator(sample_time)

        take_off_flag_leader = False
        land_flag_leader = False

        time_last = 0

        with SyncLogger(scf, lg_stab_leader) as logger_leader:
            # while not flight_terminate_flag:
            for log_entry_leader in logger_leader:
                joystick.step()
                # c_time  = time.time()
                # time_slot =  c_time - time_last
                # time_last = c_time
                # print('time',time_slot)
                # print the data from the logging list
                # timestamp = log_entry[0]

                # logconf_name = log_entry[2]

                data = log_entry_leader[1]

                # time_index = time_index + 1
                # abs_time_delayed = abs_time
                # abs_time = time.time() - start_time
                # data = receiver.get_data_sync()
                time.sleep(sample_time)
                # data = receiver.get_data()
          
                # send control command to the leader
                if joystick.get_button_l():
                    take_off_flag_leader = True

                # push the lt and rt simutaneously to land the two robots
                if joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() > axis_act_thre:
                    land_flag_leader = True

                if not take_off_flag_leader:
                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 5000)
                    # push axis_lt and x to land the leader
                    if joystick.get_button_x() and joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() < axis_deact_thre :
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break
                
                if take_off_flag_leader and not land_flag_leader:
                    # update the feedback and estimation
                    az_imu = (data['acc.y']*9.81*math.sin(math.radians(data['stateEstimate.roll'])) + data['acc.z']*9.81*math.cos(math.radians(data['stateEstimate.roll'])) - 9.81)
                    pos_z.fdk, vel_z.fdk = height_estimated.EstimatorZ(az_imu,float(data['stateEstimateZ.vz'])/1000,float(data['stateEstimateZ.z'])/1000)

                    # the desired states
                    vel_x.des = -joystick.get_axis_ly() * 0.5
                    vel_y.des = -joystick.get_axis_lx() * 0.5
                    yaw_rate.des = joystick.get_axis_rx() * 30
                    pos_z.des = pos_z.des - joystick.get_axis_ry() * sample_time * 0.3

                    # update the error
                    pos_z.err = pos_z.des - pos_z.fdk
                    vel_z.err = 0 - vel_z.fdk

                    vel_x.fdk = float(data['stateEstimateZ.vx'])/1000
                    vel_y.fdk = float(data['stateEstimateZ.vy'])/1000

                    vel_x.err = vel_x.des - vel_x.fdk
                    vel_y.err = vel_y.des - vel_y.fdk

                    # update the integrator
                    pos_z.err_inte = pos_z.err_inte + pos_z.err*sample_time
                    pos_z.err_inte = saturation_function(pos_z.err_inte, -10, 10)

                    vel_x.err_inte = vel_x.err_inte + vel_x.err*sample_time
                    vel_x.err_inte = saturation_function(vel_x.err_inte, -300, 300)

                    vel_y.err_inte = vel_y.err_inte + vel_y.err*sample_time
                    vel_y.err_inte = saturation_function(vel_y.err_inte, -300, 300)

                    # update the controller output
                    thrust_cmd_leader = pos_z_P_gain*pos_z.err + pos_z_D_gain*vel_z.err + pos_z_I_gain*pos_z.err_inte + thrust_feedforward
                    thrust_cmd_leader = int(saturation_function(thrust_cmd_leader, 0, thrust_cmd_max))

                    pitch_cmd = vel_x_P_gain*vel_x.err + vel_x_I_gain*vel_x.err_inte
                    pitch_cmd = saturation_function(pitch_cmd, -pitch_cmd_max, pitch_cmd_max)

                    roll_cmd = -(vel_y_P_gain*vel_y.err + vel_y_I_gain*vel_y.err_inte)
                    roll_cmd = saturation_function(roll_cmd, -roll_cmd_max, roll_cmd_max)

                    rec_thrust = scf.cf.commander.send_setpoint(roll_cmd, pitch_cmd, 0.0, thrust_cmd_leader)
                    
                    # print('uri',scf._link_uri,'thrust',thrust_cmd_leader)

                    # scf.cf.commander.send_hover_setpoint(vel_x.des, vel_y.des, 0.0, pos_z.des)

                    # scf.cf.commander.send_hoversetpoint

                    saver_leader_outdoor.add_elements(
                        pos_z.des, pos_z.fdk, pos_z.err, pos_z.err_inte,
                        vel_x.des, vel_x.fdk, vel_x.err, vel_x.err_inte,
                        vel_y.des, vel_y.fdk, vel_y.err, vel_y.err_inte,
                        vel_z.des, vel_z.fdk, vel_z.err, vel_z.err_inte,
                        thrust_cmd_leader,
                        pitch_cmd,
                        roll_cmd,
                        data['stabilizer.int_thrust'],
                        rec_thrust
                    )

                    # push axis_lt and x to land the leader
                    if joystick.get_button_x() and joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() < axis_deact_thre :
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break

                if land_flag_leader:
                    scf.cf.commander.send_setpoint(2.0, 0, 0, 35000)
                    # push axis_lt and x to land the leader
                    if joystick.get_button_x() and joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() < axis_deact_thre :
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break


    # follower 1 flight control branch            
    elif control_which_flag[0] == 1.0:
        lg_stab_follower_1 = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
        lg_stab_follower_1.add_variable('stabilizer.thrust', 'float')
        lg_stab_follower_1.add_variable('stabilizer.roll', 'float')
        lg_stab_follower_1.add_variable('externalforce.T', 'float')
        lg_stab_follower_1.add_variable('stabilizer.pitch', 'float')

        start_time = time.time()
        abs_time = 0.0
        follower_time_index = 0.0

        # desired thrust in Newton and roll angle for the follower
        # the value is: thrust = 2.8575 N, gamma = -7.1735 deg
        desired_thrust = 2.86
        desired_roll = -7.05

        # controller parameters for the follower
        thrust_I_gain = 25
        thrust_P_gain = 50
        
        # the feedforward term should be paid special attention
        feedforward_term = 29000
        thrust_err_limit = 0.15
        thrust_cmd_I = 12000
        thrust_cmd = 25000 # -> initial value for iteration

        take_off_flag_follower = False
        land_flag_follower = False

        with SyncLogger(scf, lg_stab_follower_1) as logger_follower_1:
            # while not flight_terminate_flag:
            for log_entry in logger_follower_1:
                # print the data from the logging list
                timestamp = log_entry[0]
                # when acc_z is positive, meaning it points downwards
                # log_acc_z = (log_entry[1].get('acc.z') - 1)*9.81
                # log_thrust = log_entry[1].get('stabilizer.thrust')
                # log_roll = log_entry[1].get('stabilizer.roll')
                # logconf_name = log_entry[2]
                force_modeled = log_entry[1].get('externalforce.T')

                abs_time_delayed = abs_time
                abs_time = time.time() - start_time
                
                time.sleep(sample_time)

                joystick.step()

                # send control command to the follower
                if joystick.get_button_r():
                    take_off_flag_follower = True

                if joystick.get_button_lb()>0.95 and joystick.get_button_rb()>0.95:
                    land_flag_follower = True
                    
                if not take_off_flag_follower:
                    # follower does not take off
                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 2000)
                    # push button X to terminate the flight when the robots is still on the ground
                    if joystick.get_button_x() and joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() < axis_deact_thre :
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break

                if take_off_flag_follower and not land_flag_follower:
                    # follower take off
                    # protection off
                    if follower_time_index <= 20:
                        scf.cf.commander.send_setpoint(0, 0, 0, 29500)
                    elif follower_time_index <= 40:
                        scf.cf.commander.send_setpoint(-1, 0, 0, 29500)
                    # elif follower_time_index <= 110:
                    #     scf.cf.commander.send_setpoint(-4, 0, 0, 31000)
                    else:
                    # if not position_control_flag:
                        # the feedforward + PI control for force, (model referenced)
                        thrust_error_inNewton = desired_thrust - log_entry[1].get('externalforce.T')

                        if thrust_error_inNewton >= thrust_err_limit:
                            thrust_error_inNewton = thrust_err_limit
                        if thrust_error_inNewton <= -thrust_err_limit:
                            thrust_error_inNewton = -thrust_err_limit

                        thrust_cmd_I = int(thrust_cmd_I + thrust_I_gain*thrust_error_inNewton)

                        if thrust_cmd_I >= 18000:
                            thrust_cmd_I = 18000

                        thrust_cmd = thrust_cmd_I + thrust_error_inNewton*thrust_P_gain + feedforward_term

                        thrust_cmd = limit_thrust(thrust_cmd)

                        if thrust_cmd <= 35000:
                            thrust_cmd = int(35000)

                        # this is to prevent too much force in the taking off stage
                        if follower_time_index <= 3000:
                            if thrust_cmd >= 45000:
                                thrust_cmd = int(45000)
                        
                        scf.cf.commander.send_setpoint(desired_roll, 0, 0, thrust_cmd)
                        
                        if follower_time_index % 50 == 0:
                            print('position_control_off', follower_time_index/50, 'sec', ' ,thrust_cmd=', thrust_cmd,
                            ' ,roll_desired=', desired_roll, ' ,roll_actual = ', log_entry[1].get('stabilizer.roll'),
                            ' ,force_modeled = ', force_modeled)

                    if joystick.get_button_x() and joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() < axis_deact_thre :
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break

                    follower_time_index = follower_time_index + 1

                
                if land_flag_follower:
                    scf.cf.commander.send_setpoint(-5, 0, 0, 35000)
                    if joystick.get_button_x() and joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() < axis_deact_thre :
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break
        

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # # set the sample here, hz
    sample_rate = 100
    sample_time = 1 / sample_rate
    sample_rate_in_ms = int(sample_time * 1000)
    # receiver.start_thread()

    # set the URI of the quadrotors which you wish to control in the swarm

    uri_leader = 'radio://0/31/2M'  
    uri_follower_1 = 'radio://0/101/2M'

    control_flag_args = {
        uri_leader: [[0.0, 0.0, 0.0]],
        uri_follower_1: [[1.0, 1.0, 1.0]]
    }

    URIS_swarm = {uri_leader, uri_follower_1}


    factory = CachedCfFactory(rw_cache='./cache')

    # data saver
    saver_leader_outdoor = savemat.DataSaver(
        'pos_z_des', 'pos_z_fdk', 'pos_z_err', 'pos_z_err_inte',
        'vel_x_des', 'vel_x_fdk', 'vel_x_err', 'vel_x_err_inte',       
        'vel_y_des', 'vel_y_fdk', 'vel_y_err', 'vel_y_err_inte',
        'vel_z_des', 'vel_z_fdk', 'vel_z_err', 'vel_z_err_inte',
        'thrust_cmd_leader',
        'pitch_cmd',
        'roll_cmd',
        'thrust_int',
        'rec_thrust'
    )

    # starts the swarm!!!
    with Swarm(URIS_swarm, factory=factory) as swarm:
        swarm.parallel_safe(crazyflie_control, args_dict = control_flag_args)
        
        
    saver_leader_outdoor.save2mat('DATA_outdoor_flight/')

    # receiver.stop_thread()
    # saver.save2mat('DataExchange/')
    # saver_follower_1.save2mat('DATA_leader_optical_flow/')

    joystick.quit()

