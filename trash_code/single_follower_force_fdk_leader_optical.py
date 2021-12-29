"""
This is the program fot the swarm of leader-follower cooperative transport

the leader and follower will use the MoCap to reach a proper relative position
then the position control of the follower will be turned off, using the force given 
by the model as reference

both the desired force and desired roll angle are calculated by the equilibrium condition

# the controller for the thrust should be modified in this programe



------!!!-----
NOV 1, 2021
the thrust controller for the follower is still use the logged data as feedback
which is unprecise and has delay

now the firmware has been modified that we can send thrust in Newton in Python

so the code is of not too much use
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
from cooperative_transport.controllers import PID_ControllerThreeAixs
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
    start_time = time.time()
    abs_time = 0.0

    time_index = 0

    follower_time_index = 0.0

    # controller parameter and taking off position set for the leader and follower
    desired_x_leader = 0.0
    desired_y_leader = 0.0
    desired_z_leader = 0.1
    desired_yaw_rate_leader = 0.0

    desired_x_follower = 1.16
    desired_y_follower = -0.83
    desired_z_follower = 0.03
    desired_yaw_follower = 0.0

    desired_vx_leader = 0.0
    desired_vy_leader = 0.0
    desired_yaw_rate = 0.0
    desired_height = 0.2
    
    # the logging data list, for the leader
    lg_stab_leader = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
    lg_stab_leader.add_variable('acc.z', 'float')
    lg_stab_leader.add_variable('stabilizer.thrust', 'float')
    lg_stab_leader.add_variable('stabilizer.roll', 'float')

    # the logging date list, for the follower 1
    lg_stab_follower_1 = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
    lg_stab_follower_1.add_variable('stabilizer.thrust', 'float')
    lg_stab_follower_1.add_variable('stabilizer.roll', 'float')
    lg_stab_follower_1.add_variable('externalforce.T', 'float')
    lg_stab_follower_1.add_variable('stabilizer.pitch', 'float')



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

    position_controller_follower =  PID_ControllerThreeAixs(sample_time,
                                                  10, 1, 20, 0,
                                                  7, 1, 20, 0,
                                                  10000, 500, 10000, 40000, ) 

    # if yaw_constant_flag is Ture, then the desired yaw will set to be 0 for the whole flight
    yaw_constant_flag_leader = True
    yaw_constant_flag_follower = True

    # take off flag and land flag for the leader and follower
    take_off_flag_leader = False
    take_off_flag_follower = False

    land_flag_leader = False
    land_flag_follower = False

    # position control flag of the follower 1
    position_control_flag = False


    # desired commands, calculated by the equilibrium condition

    # for m_payload = 0.126 kg, and m_follower = 0.226 kg,
    # alpha_0 = 60 deg, alpha_1 = 60 deg, ->

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
    thrust_I_gain = 25
    thrust_P_gain = 50
    # the feedforward term should be paid special attention
    feedforward_term = 29000

    thrust_err_limit = 0.15

    # thrust_base = 39500

    thrust_base = 25000

    thrust_cmd_I = 12000

    thrust_cmd = thrust_base # -> initial value for iteration

    # --------------------------------------------------------------------------------
    
    thrust_count = 0
    roll_count = 0

    thrust_inc_flag = False
    thrust_dec_flag = False
    roll_inc_flag = False
    roll_dec_flag = False

    axis_act_thre = 0.97
    axis_deact_thre = 0.03

    # ------- leader control branch -------
    if control_which_flag[0] == 0.0:
        with SyncLogger(scf, lg_stab_leader) as logger_leader:
            # while not flight_terminate_flag:
            for log_entry in logger_leader:
                # print the data from the logging list
                timestamp = log_entry[0]

                logconf_name = log_entry[2]

                time_index = time_index + 1
                abs_time_delayed = abs_time
                abs_time = time.time() - start_time
                # data = receiver.get_data_sync()
                time.sleep(sample_time)
                data = receiver.get_data()

                # digid body ID of the leader from the MoCap
                dp_leader.step(data[(rigid_body_ID_leader-1)*14 : rigid_body_ID_leader*14])
                joystick.step()

                # get the feedback value of the leader
                # yaw_feedback_leader = math.atan2(2 * (dp_leader.QW * dp_leader.QZ + dp_leader.QX * dp_leader.QY), 1 - 2 * (dp_leader.QZ * dp_leader.QZ + dp_leader.QY * dp_leader.QY))
                # X_feedback_leader = Filter_x.filter(dp_leader.X)
                # Y_feedback_leader = Filter_y.filter(dp_leader.Y)
                # Z_feedback_leader = Filter_z.filter(dp_leader.Z)

                # the position control of the leader is activated when axis_lt is pushed and axis_rt is released
                if joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() < axis_deact_thre:
                    # desired_x_leader = desired_x_leader + joystick.get_axis_ly() * sample_time * 0.5
                    # desired_y_leader = desired_y_leader + joystick.get_axis_lx() * sample_time * 0.5
                    # desired_z_leader = desired_z_leader - joystick.get_axis_ry() * sample_time * 0.4
                    # if yaw_constant_flag_leader:
                    #     desired_yaw_leader = 0.0
                    # else:
                    #     desired_yaw_leader = Filter_pad_rx.filter(desired_yaw_leader + joystick.get_axis_rx() * sample_time * 20)

                    # the desired position is now given by the optical flow
                    # desired_x_leader = desired_x_leader - joystick.get_axis_ly() * sample_time * 0.5
                    # desired_y_leader = desired_y_leader - joystick.get_axis_lx() * sample_time * 0.5
                    # desired_yaw_rate_leader = joystick.get_axis_rx() * 90

                    # desired_z_leader = desired_z_leader - joystick.get_axis_ry() * sample_time * 0.4

                    desired_vx_leader = -joystick.get_axis_ly() * 0.5
                    desired_vy_leader = -joystick.get_axis_lx() * 0.5
                    desired_yaw_rate_leader = joystick.get_axis_rx() * 30
                    desired_height = desired_height - joystick.get_axis_ry() * sample_time * 0.4


                # # update the reference and get the controller output for the leader
                # position_controller_leader.update_reference(desired_x_leader, desired_y_leader, desired_z_leader)

                # u_x_leader, u_y_leader, u_z_leader = position_controller_leader.update_error(X_feedback_leader, Y_feedback_leader, Z_feedback_leader)

                # # mapping the roll, pithch, yaw and thrust for the leader
                # pitch_leader = -(u_x_leader * math.cos(yaw_feedback_leader) + u_y_leader * math.sin(yaw_feedback_leader)) / (math.cos(yaw_feedback_leader) * math.cos(yaw_feedback_leader) 
                #           + math.sin(yaw_feedback_leader) * math.sin(yaw_feedback_leader))
                # roll_leader = (u_y_leader * math.cos(yaw_feedback_leader) - u_x_leader * math.sin(yaw_feedback_leader)) / (math.cos(yaw_feedback_leader) * math.cos(yaw_feedback_leader) 
                #         + math.sin(yaw_feedback_leader) * math.sin(yaw_feedback_leader))
                # thrust_leader = limit_thrust(u_z_leader)

                # Yaw_error_leader = limit_angular(limit_angular(desired_yaw_leader) - limit_angular(yaw_feedback_leader))
                # yaw_leader = - Yaw_error_leader * 50
                
               

                # send control command to the leader
                if joystick.get_button_l():
                    take_off_flag_leader = True

                # push the lt and rt simutaneously to land the two robots
                if joystick.get_button_lb()>0.95 and joystick.get_button_rb()>0.95:
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
                    # scf.cf.commander.send_position_setpoint(desired_x_leader, desired_y_leader, 
                    # desired_z_leader, desired_yaw_rate_leader)
                    scf.cf.commander.send_hover_setpoint(desired_vx_leader, desired_vy_leader, 
                    desired_yaw_rate_leader, desired_height)

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
                if joystick.get_button_y() and joystick.get_button_lb() < axis_deact_thre:
                    position_control_flag = False

                # turn on the position control
                if joystick.get_button_a()  and joystick.get_button_lb() < axis_deact_thre:
                    position_control_flag = True

                # send control command to the follower
                if joystick.get_button_r():
                    take_off_flag_follower = True

                if joystick.get_button_lb()>0.95 and joystick.get_button_rb()>0.95:
                    land_flag_follower = True
                    
                if not take_off_flag_follower:
                    # follower does not take off
                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 5000)
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
                        scf.cf.commander.send_setpoint(-2, 0, 0, 29500)
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
                        if follower_time_index <= 6000:
                            if thrust_cmd >= 45000:
                                thrust_cmd = int(45000)
                        
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

                        if follower_time_index % 50 == 0:
                            print('position_control_off', follower_time_index/50, 'sec', ' ,thrust_cmd=', thrust_cmd,
                            ' ,roll_desired=', desired_roll, ' ,roll_actual = ', log_entry[1].get('stabilizer.roll'),
                            ' ,force_modeled = ', force_modeled)
                    # else:
                    #     # position control on
                    #     scf.cf.commander.send_setpoint(roll_follower, pitch_follower, yaw_follower, thrust_follower)

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
    saver_follower_1.save2mat('DATA_leader_optical_flow/')

    joystick.quit()

