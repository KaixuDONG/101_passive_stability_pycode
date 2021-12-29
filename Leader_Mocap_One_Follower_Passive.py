"""
This is the program fot the swarm of leader-follower cooperative transport

both the states of the leader, the follower and the payload are recorded by the Mocap
the leader is controlled by the Mocap
while the follower totally depends on the passive stability

the take off time and land time is recorded as well

the version 1.2 is finished in AUG 28, 2021 by Estel


---!!!----
Nov 1, 2021
the thrust control for the follower in this code
is STILL by the logged data as feedback

not the online, firmware control

so be cautious!

pay special attention to the thrust
and the commnand of 'send_setpoint()'
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

# libs from Estel
from cooperative_transport.controllers import PID_ControllerThreeAixs, Creat_Agent
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
def crazyflie_control(scf, control_which_flag):
    """
    set the controller for this carzyflie in the firmware
    1 -> PID controller
    2 -> Mellinger controller
    3 -> INDI controller
    """
    # set the controller for all the crazyflies in the swarm to be PID 
    scf.cf.param.set_value('stabilizer.controller', '1')
    time.sleep(0.1)
    # unlock all the motors, this step is very significant
    scf.cf.commander.send_setpoint(0, 0, 0, 0)
    time.sleep(0.1)

    # data processor, filtering the data from MoCap and command from joystick
    dp_leader = GeneralFcn.RealTimeProcessor()
    dp_follower = GeneralFcn.RealTimeProcessor()
    dp_payload = GeneralFcn.RealTimeProcessor()

    # set the rigidbody ID
    rigid_body_ID_leader = 1
    rigid_body_ID_follower_1 = 2
    rigid_body_ID_payload = 3

    Filter_x = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)
    Filter_y = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)
    Filter_z = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)

    Filter_pad_ly = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=65, fs=sample_rate)
    Filter_pad_lx = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=65, fs=sample_rate)
    Filter_pad_ry = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=65, fs=sample_rate)
    Filter_pad_rx = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=50, fs=sample_rate)

    # controller parameter and taking off position set for the leader and follower
    trajectory_on = True
    position_controller_leader = PID_ControllerThreeAixs(sample_time,
                                                  10, 1, 20, 0,
                                                  7, 1, 20, 0,
                                                  10000, 500, 10000, 40000, )

    # position_controller_follower =  PID_ControllerThreeAixs(sample_time,
    #                                               10, 1, 20, 0,
    #                                               7, 1, 20, 0,
    #                                               10000, 500, 10000, 40000, ) 

    # if yaw_constant_flag is Ture, then the desired yaw will set to be 0 for the whole flight
    yaw_constant_flag_leader = True
    yaw_constant_flag_follower = True

    # take off flag and land flag for the leader and follower
    # take_off_flag_leader = False
    # take_off_flag_follower = False

    # land_flag_leader = False
    # land_flag_follower = False

    # position control flag of the follower 1
    position_control_flag = False


    # desired commands, calculated by the equilibrium condition

    # for m_payload = 0.126 kg, and m_follower = 0.226 kg,
    # alpha_0 = 60 deg, alpha_1 = 60 deg, ->

    # # the value is: thrust = 2.8575 N, gamma = -7.1735 deg
    # desired_thrust = 2.85
    # desired_roll = -7.1

    # # thrust_p_gain = 30
    # # thrust_d_gain = 50
    
    # # coefficient set 1 -> good at the begining, but diverge after 60 sec
    # # thrust_I_gain = 60
    # # thrust_P_gain = 30
    # # feedforward_term = 8000

    # # coefficient set 2 -> can not track as closely as set 1, but wont diverge in the whole 2 min flight
    # # thrust_I_gain = 30
    # # thrust_P_gain = 50
    # # feedforward_term = 9000

    # # thrust_I_gain = 25
    # thrust_I_gain = 25
    # thrust_P_gain = 50
    # # the feedforward term should be paid special attention
    # feedforward_term = 29000

    # thrust_err_limit = 0.15

    # # thrust_base = 39500

    # thrust_base = 25000

    # thrust_cmd_I = 12000

    # thrust_cmd = thrust_base # -> initial value for iteration

    # --------------------------------------------------------------------------------
    
    # thrust_count = 0
    # roll_count = 0

    # thrust_inc_flag = False
    # thrust_dec_flag = False
    # roll_inc_flag = False
    # roll_dec_flag = False

    axis_act_thre = 0.95
    axis_deact_thre = 0.05


    # ------- leader control branch -------
    if control_which_flag[0] == 0.0:
        # the logging data list, for the leader
        lg_stab_leader = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
        lg_stab_leader.add_variable('acc.z', 'float')
        lg_stab_leader.add_variable('stabilizer.thrust', 'float')
        lg_stab_leader.add_variable('stabilizer.roll', 'float')

        with SyncLogger(scf, lg_stab_leader) as logger_leader:
            # while not flight_terminate_flag:
            for log_entry in logger_leader:
                # print the data from the logging list
                # timestamp = log_entry[0]

                # logconf_name = log_entry[2]

                # time_index = time_index + 1
                # abs_time_delayed = abs_time
                # abs_time = time.time() - start_time
                # data = receiver.get_data_sync()
                
                time.sleep(sample_time)
                data = receiver.get_data()

                # digid body ID of the leader from the MoCap
                dp_leader.step(data[(rigid_body_ID_leader-1)*14 : rigid_body_ID_leader*14])
                joystick.step()

                # get the feedback value of the leader
                leader.yaw.fdk = math.atan2(2 * (dp_leader.QW * dp_leader.QZ + dp_leader.QX * dp_leader.QY), 1 - 2 * (dp_leader.QZ * dp_leader.QZ + dp_leader.QY * dp_leader.QY))
                leader.pos_x.fdk = Filter_x.filter(dp_leader.X)
                leader.pos_y.fdk = Filter_y.filter(dp_leader.Y)
                leader.pos_z.fdk = Filter_z.filter(dp_leader.Z)

                # the position control of the leader is activated when both LT and RT are released
                if joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() < axis_deact_thre:
                    leader.pos_x.des = leader.pos_x.des + joystick.get_axis_ly() * sample_time * 0.5
                    leader.pos_y.des = leader.pos_y.des + joystick.get_axis_lx() * sample_time * 0.5
                    leader.pos_z.des = leader.pos_z.des - joystick.get_axis_ry() * sample_time * 0.4
                    if yaw_constant_flag_leader:
                        leader.yaw.des = 0.0
                    else:
                        leader.yaw.des = Filter_pad_rx.filter(leader.yaw.des + joystick.get_axis_rx() * sample_time * 20)
                
                # update the reference and get the controller output for the leader
                position_controller_leader.update_reference(leader.pos_x.des, leader.pos_y.des, leader.pos_z.des)

                u_x_leader, u_y_leader, u_z_leader = position_controller_leader.update_error(leader.pos_x.fdk, leader.pos_y.fdk, leader.pos_z.fdk)

                # mapping the roll, pithch, yaw and thrust for the leader
                leader.pitch.ctrl_cmd = -(u_x_leader * math.cos(leader.yaw.fdk) + u_y_leader * math.sin(leader.yaw.fdk)) / (math.cos(leader.yaw.fdk) * math.cos(leader.yaw.fdk) 
                          + math.sin(leader.yaw.fdk) * math.sin(leader.yaw.fdk))
                leader.roll.ctrl_cmd = (u_y_leader * math.cos(leader.yaw.fdk) - u_x_leader * math.sin(leader.yaw.fdk)) / (math.cos(leader.yaw.fdk) * math.cos(leader.yaw.fdk) 
                        + math.sin(leader.yaw.fdk) * math.sin(leader.yaw.fdk))
                leader.thrus_cmd.ctrl_cmd = limit_thrust(u_z_leader)

                leader.yaw.err = limit_angular(limit_angular(leader.yaw.des) - limit_angular(leader.yaw.fdk))
                leader.yaw.ctrl_cmd = -leader.yaw.err * 50
                
                # print the current position of the quadrotor to check
                # if time_index % 100 == 0:
                #     print(time_index/100,'sec,', ' leader position is', X_feedback_leader, Y_feedback_leader, Z_feedback_leader)

                # send control command to the leader
                if joystick.get_button_l():
                    leader.take_off_flag = True
                    leader.event_flag = int(1)
                    leader.take_off_time = time.time()

                # land the formation when the LT and RT are pushed
                if joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() > axis_act_thre:
                    leader.land_flag = True
                    leader.event_flag = int(2)
                    leader.land_time = time.time()

                if not leader.take_off_flag:
                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 5000)
                    # push axis_lt and x to land the leader
                    if joystick.get_button_x() and joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() < axis_deact_thre :
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break
                
                if leader.take_off_flag and not leader.land_flag:
                    scf.cf.commander.send_setpoint(leader.roll.ctrl_cmd, leader.pitch.ctrl_cmd, leader.yaw.ctrl_cmd, leader.thrus_cmd.ctrl_cmd)
                    # push axis_lt and x to land the leader
                    if joystick.get_button_x() and joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() < axis_deact_thre :
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break

                if leader.land_flag:
                    if time.time() - leader.land_time <= 2.0:
                        scf.cf.commander.send_setpoint(2.0, 0, 0, 33000)
                    elif time.time() - leader.land_time <= 4.0:
                        scf.cf.commander.send_setpoint(0.0, 0, 0, 31000)
                    elif time.time() - leader.land_time <= 6.0:
                        scf.cf.commander.send_setpoint(0.0, 0, 0, 29000)
                    else:
                        scf.cf.commander.send_setpoint(0.0, 0, 0, 5000)


                    # push axis_lt and x to land the leader
                    if joystick.get_button_x() and joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() < axis_deact_thre :
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break


    # follower 1 flight control branch            
    elif control_which_flag[0] == 1.0:

        # the value is: thrust = 2.8575 N, gamma = -7.1735 deg
        desired_thrust = 2.85
        desired_roll = -7.1

        # thrust_p_gain = 30
        # thrust_d_gain = 50
        
        # coefficient set 1 -> good at the begining, but diverge after 60 sec
        # thrust_I_gain = 60
        # thrust_P_gain = 30
        # feedforward_term = 8000

        # coefficient set 2 -> can not track as closely as set 1, but wont diverge in the whole 2 min flight
        # thrust_I_gain = 30
        # thrust_P_gain = 50
        # feedforward_term = 9000

        # thrust_I_gain = 25
        thrust_I_gain = 27
        thrust_P_gain = 47
        # the feedforward term should be paid special attention
        feedforward_term = 28500

        thrust_err_limit = 0.15

        # thrust_base = 39500

        # thrust_base = 25000

        thrust_cmd_I = 12000

        # thrust_cmd = thrust_base # -> initial value for iteration


        # the logging date list, for the follower 1
        lg_stab_follower_1 = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
        lg_stab_follower_1.add_variable('stabilizer.thrust', 'float')
        lg_stab_follower_1.add_variable('stabilizer.roll', 'float')
        lg_stab_follower_1.add_variable('externalforce.T', 'float')
        lg_stab_follower_1.add_variable('stabilizer.pitch', 'float')

        with SyncLogger(scf, lg_stab_follower_1) as logger_follower_1:
            # while not flight_terminate_flag:
            for log_entry in logger_follower_1:
                # print the data from the logging list
                # timestamp = log_entry[0]
                # when acc_z is positive, meaning it points downwards
                # log_acc_z = (log_entry[1].get('acc.z') - 1)*9.81
                # log_thrust = log_entry[1].get('stabilizer.thrust')
                # log_roll = log_entry[1].get('stabilizer.roll')
                # logconf_name = log_entry[2]
                force_modeled = log_entry[1].get('externalforce.T')

                # abs_time_delayed = abs_time
                # abs_time = time.time() - start_time
                # data = receiver.get_data_sync()
                
                time.sleep(sample_time)
                data = receiver.get_data()

                # digid body ID of the leader from the MoCap
                dp_leader.step(data[(rigid_body_ID_leader-1)*14 : rigid_body_ID_leader*14])

                dp_follower.step(data[(rigid_body_ID_follower_1-1)*14 : rigid_body_ID_follower_1*14])

                dp_payload.step(data[(rigid_body_ID_payload-1)*14 : rigid_body_ID_payload*14])

                joystick.step()

                # do not need the initial position control for the follower

                # # get the feedback value of the leader
                # yaw_feedback_follower = math.atan2(2 * (dp_follower.QW * dp_follower.QZ + dp_follower.QX * dp_follower.QY), 1 - 2 * (dp_follower.QZ * dp_follower.QZ + dp_follower.QY * dp_follower.QY))
                # X_feedback_follower = Filter_x.filter(dp_follower.X)
                # Y_feedback_follower = Filter_y.filter(dp_follower.Y)
                # Z_feedback_follower = Filter_z.filter(dp_follower.Z)

                # # the position control of the follower is activated when both axis_lt and axis_rt are released
                # if joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() < axis_deact_thre:
                #     desired_x_follower = desired_x_follower + joystick.get_axis_ly() * sample_time * 0.5
                #     desired_y_follower = desired_y_follower + joystick.get_axis_lx() * sample_time * 0.5
                #     desired_z_follower = desired_z_follower - joystick.get_axis_ry() * sample_time * 0.4
                #     if yaw_constant_flag_follower:
                #         desired_yaw_follower = 0.0
                #     else:
                #         desired_yaw_follower = Filter_pad_rx.filter(desired_yaw_follower + joystick.get_axis_rx() * sample_time * 20)
                
                # # update the reference and get the controller output for the follower 1
                # position_controller_follower.update_reference(desired_x_follower, desired_y_follower, desired_z_follower)

                # u_x_follower, u_y_follower, u_z_follower = position_controller_follower.update_error(X_feedback_follower, Y_feedback_follower, Z_feedback_follower)

                # # mapping the roll, pithch, yaw and thrust for the leader
                # pitch_follower = -(u_x_follower * math.cos(yaw_feedback_follower) + u_y_follower * math.sin(yaw_feedback_follower)) / (math.cos(yaw_feedback_follower) * math.cos(yaw_feedback_follower) 
                #           + math.sin(yaw_feedback_follower) * math.sin(yaw_feedback_follower))
                # roll_follower = (u_y_follower * math.cos(yaw_feedback_follower) - u_x_follower * math.sin(yaw_feedback_follower)) / (math.cos(yaw_feedback_follower) * math.cos(yaw_feedback_follower) 
                #         + math.sin(yaw_feedback_follower) * math.sin(yaw_feedback_follower))
                # thrust_follower = limit_thrust(u_z_follower)

                # Yaw_error_follower = limit_angular(limit_angular(desired_yaw_follower) - limit_angular(yaw_feedback_follower))
                # yaw_follower = - Yaw_error_follower * 50

                # # to adjust the roll and thrust commands by the joystick when the position control is off
                # thrust_inc_flag_prev = thrust_inc_flag
                # thrust_dec_flag_prev = thrust_dec_flag
                # roll_inc_flag_prev = roll_inc_flag
                # roll_dec_flag_prev = roll_dec_flag

                # # rb -> increase the thrust
                # if joystick.get_button_rb() and joystick.get_button_lb() < axis_deact_thre:
                #     thrust_inc_flag = True
                #     if (not thrust_inc_flag_prev) and  thrust_inc_flag:
                #         thrust_count = int(thrust_count + 1)
                # else:
                #     thrust_inc_flag = False 

                # # lb -> decrease the thrust
                # if joystick.get_button_lb() and joystick.get_button_lb() < axis_deact_thre:
                #     thrust_dec_flag = True
                #     if (not thrust_dec_flag_prev) and  thrust_dec_flag:
                #         thrust_count = int(thrust_count - 1)
                # else:
                #     thrust_dec_flag = False    

                # # b -> increase the roll
                # if joystick.get_button_b() and joystick.get_button_lb() < axis_deact_thre:
                #     roll_inc_flag = True
                #     if (not roll_inc_flag_prev) and  roll_inc_flag:
                #         roll_count = int(roll_count + 1)
                # else:
                #     roll_inc_flag = False 

                # # a -> decrease the roll
                # if joystick.get_button_a() and joystick.get_button_lb() < axis_deact_thre:
                #     roll_dec_flag = True
                #     if (not roll_dec_flag_prev) and  roll_dec_flag:
                #         roll_count = int(roll_count - 1)
                # else:
                #     roll_dec_flag = False

                # turn off the position control, push button Y, release the axis_lt
                if joystick.get_button_y() and joystick.get_button_lb() > axis_act_thre:
                    position_control_flag = False

                # turn on the position control
                if joystick.get_button_a()  and joystick.get_button_lb() > axis_act_thre:
                    position_control_flag = True

                # send control command to the follower
                if joystick.get_button_r():
                    follower_1.take_off_flag = True
                    follower_1.event_flag = int(1)
                    follower_1.take_off_time = time.time()

                # land the formation when the LT and RT are pushed
                if joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() > axis_act_thre:
                    follower_1.land_flag = True
                    follower_1.event_flag = int(2)
                    follower_1.land_time = time.time()

                if not follower_1.take_off_flag:
                    # follower does not take off
                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 5000)
                    # push button X to terminate the flight when the robots is still on the ground
                    if joystick.get_button_x() and joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() < axis_deact_thre :
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break

                if follower_1.take_off_flag and not follower_1.land_flag:
                    # follower take off
                    # protection off
                    if (time.time() - follower_1.take_off_time) <= 0.3:
                        scf.cf.commander.send_setpoint(0, 0, 0, 29500)
                    elif (time.time() - follower_1.take_off_time) <= 0.6:
                        scf.cf.commander.send_setpoint(-2, 0, 0, 29500)
                    # elif follower_time_index <= 110:
                    #     scf.cf.commander.send_setpoint(-4, 0, 0, 31000)
                    else:
                    # if not position_control_flag:
                        # the feedforward + PI control for force, (model referenced)

                        # varying formation
                        if (time.time() - follower_1.take_off_time) <= 30:
                            # alpha_0 = pi/3, alpha_1 = pi/3
                            desired_thrust = 2.85
                            desired_roll = -7.1
                        elif (time.time() - follower_1.take_off_time) <= 60:
                            # alpha_0 = pi/3, aplha_1 = pi/4
                            desired_thrust = 2.704
                            desired_roll = -9.4      
                        elif (time.time() - follower_1.take_off_time) <= 90:  
                            # alpha_0 = pi/4, alpha_1 = pi/4                   
                            desired_thrust = 2.8938
                            desired_roll = -12.0337  
                        else:
                            # alpha_0 = pi/3, alpha_1 = pi/3
                            desired_thrust = 2.85
                            desired_roll = -7.1                            

                        thrust_error_inNewton = desired_thrust - log_entry[1].get('externalforce.T')

                        if thrust_error_inNewton >= thrust_err_limit:
                            thrust_error_inNewton = thrust_err_limit
                        if thrust_error_inNewton <= -thrust_err_limit:
                            thrust_error_inNewton = -thrust_err_limit

                        thrust_cmd_I = int(thrust_cmd_I + thrust_I_gain*thrust_error_inNewton)

                        if thrust_cmd_I >= 20000:
                            thrust_cmd_I = 20000

                        thrust_cmd = thrust_cmd_I + thrust_error_inNewton*thrust_P_gain + feedforward_term

                        thrust_cmd = limit_thrust(thrust_cmd)

                        if thrust_cmd <= 33000:
                            thrust_cmd = int(33000)

                        # this is to prevent too much force in the taking off stage
                        if (time.time() - follower_1.take_off_time) <= 5.0:
                            if thrust_cmd >= 43000:
                                thrust_cmd = int(43000)
                        
                        scf.cf.commander.send_setpoint(desired_roll, 0, 0, thrust_cmd)
                        
                        # thrust_error_inNewton_prev = thrust_error_inNewton
                        
                        # only log when the position control is off
                        saver_follower_1.add_elements(log_entry[1].get('stabilizer.roll'), 
                            log_entry[1].get('stabilizer.pitch'),
                            log_entry[1].get('externalforce.T'),
                            dp_leader.X, dp_leader.Y, dp_leader.Z,
                            dp_follower.X, dp_follower.Y, dp_follower.Z, 
                            dp_payload.X, dp_payload.Y, dp_payload.Z,
                            thrust_cmd
                            )

                        # if follower_time_index % 50 == 0:
                        #     print('position_control_off', follower_time_index/50, 'sec', ' ,thrust_cmd=', thrust_cmd,
                        #     ' ,roll_desired=', desired_roll, ' ,roll_actual = ', log_entry[1].get('stabilizer.roll'),
                        #     ' ,force_modeled = ', force_modeled)
                    # else:
                    #     # position control on
                    #     scf.cf.commander.send_setpoint(roll_follower, pitch_follower, yaw_follower, thrust_follower)

                    if joystick.get_button_x() and joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() < axis_deact_thre :
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break

                    # follower_time_index = follower_time_index + 1

                
                if follower_1.land_flag:
                    if (time.time() - follower_1.land_time <= 1.1):
                        scf.cf.commander.send_setpoint(-5, 0, 0, 35000)
                    elif (time.time() - follower_1.land_time <= 2.3):
                        scf.cf.commander.send_setpoint(0, 0, 0, 28000)
                    else:
                        scf.cf.commander.send_setpoint(0, 0, 0, 24000)

                    if joystick.get_button_x() and joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() < axis_deact_thre :
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break

                follower_1.event_flag_prev = follower_1.event_flag
        

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # my_trajectory = trajectory_loader.Trajectory('/CFrobot/DataExchange/Trajecotries/T1.mat')
    # my_trajectory.update()
    # udp receiver
    receiver = UdpReceiver.UdpRigidBodies()
    # # sample_rate = receiver.get_sample_rate()
    # # set the sample here, hz
    sample_rate = 100
    sample_time = 1 / sample_rate
    sample_rate_in_ms = int(sample_time * 1000)
    receiver.start_thread()

    # creat agents
    leader = Creat_Agent()
    follower_1 = Creat_Agent()
    payload = Creat_Agent()

    # set the initial position of the agents
    leader.pos_x.des = 0.41
    leader.pos_y.des = 0.48
    leader.pos_z.des = 0.07
    
    # set the URI of the quadrotors which you wish to control in the swarm

    #uri_leader = 'radio://0/23/2M/E7E7E7E701'  
    #uri_follower_1 = 'radio://0/101/2M/E7E7E7E702'
    uri_leader = 'radio://0/31/2M'  
    uri_follower_1 = 'radio://0/101/2M'

    control_flag_args = {
        uri_leader: [[0.0, 0.0, 0.0]],
        uri_follower_1: [[1.0, 1.0, 1.0]]
    }

    URIS_swarm = {uri_leader, uri_follower_1}

    # logging.basicConfig(level=logging.ERROR)

    # flight_terminate_flag = False

    factory = CachedCfFactory(rw_cache='./cache')
    
    saver_follower_1 = savemat.DataSaver('roll_actual', 'pitch_actual', 'thrust_actual', 
        'X_pos_leader', 'Y_pos_leader', 'Z_pos_leader',
        'X_pos_follower_1', 'Y_pos_follower_1', 'Z_pos_follower_1',
        'X_pos_payload', 'Y_pos_payload', 'Z_pos_payload', 'thrust_cmd')


    # starts the swarm!!!
    with Swarm(URIS_swarm, factory=factory) as swarm:
        swarm.parallel_safe(crazyflie_control, args_dict = control_flag_args)
    
    receiver.stop_thread()
    # saver.save2mat('DataExchange/')
    saver_follower_1.save2mat('DATA_Leader_Mocap_One_Follower_Passive/')

    joystick.quit()

