"""
This is the program fot the swarm of leader-follower cooperative transport

the leader uses optical flow to replece the role of mocap 
while the follower uses the mocap to control its states


------!!!!!----------
the code is of limited usage
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

from cooperative_transport.controllers import EKF_Estimator
from cooperative_transport.controllers import Sys_State_def
from cooperative_transport.controllers import saturation_function


# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# saturation funciton for thrust
def limit_thrust(thrust_cmd):
    if thrust_cmd > 65535:
        thrust_cmd = 65535
    if thrust_cmd < 0:
        thrust_cmd = 0
    return int(thrust_cmd)

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
    scf.cf.commander.send_setpoint(0, 0, 0, 0)
    time.sleep(0.1)
    start_time = time.time()
    abs_time = 0.0

    time_index = 0

    # controller parameter and taking off position set for the leader and follower
    #trajectory_on = True                                         
    desired_x_leader = 1.28
    desired_y_leader = -0.37
    desired_z_leader = 0.2
    desired_yaw_leader = 0.0

    vel_x = Sys_State_def()
    vel_y = Sys_State_def()
    vel_z = Sys_State_def()
    yaw_rate = Sys_State_def()
    pos_z = Sys_State_def()
    pos_z.des = desired_z_leader

    desired_x_follower = 0.44
    desired_y_follower = 1.04
    desired_z_follower = 0.03
    desired_yaw_follower = 0.0
    
    # the logging data list
    # lg_stab = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
    # # lg_stab.add_variable('stabilizer.roll', 'float')
    # # lg_stab.add_variable('stabilizer.pitch', 'float')
    # # lg_stab.add_variable('stabilizer.yaw', 'float')
    # lg_stab.add_variable('acc.z', 'float')
    # lg_stab.add_variable('stabilizer.thrust', 'float')
    # lg_stab.add_variable('stabilizer.roll', 'float')



    # data processor, filtering the data from MoCap and command from joystick
    dp_leader = GeneralFcn.RealTimeProcessor()
    dp_follower = GeneralFcn.RealTimeProcessor()

    # set the rigidbody ID
    rigid_body_ID_leader = 1
    rigid_body_ID_follower_1 = 2

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

    # take off flag for the leader and follower
    take_off_flag_leader = False
    take_off_flag_follower = False

    # land_flag_leader = False
    # land_flag_follower = False

    # # parameter and command set for the follower
    # # set the inertia parameters
    # inertia_dict = {
    #     'g': 9.81,
    #     'mp': 84.5/1000,   #gram, mass of the payload
    #     'mf': (216 + 10)/1000,  #gram, mass of the follower
    #     'alpha_1': math.pi/3,  #rad
    #     'alpha_2': math.pi/3,  #rad
    # }

    # g = inertia_dict['g']
    # mp =  inertia_dict['mp']
    # mf =  inertia_dict['mf']
    # alpha_1 = inertia_dict['alpha_1']
    # alpha_2 = inertia_dict['alpha_2']

    # T2 = 1/(math.cos(alpha_2)/math.cos(alpha_1)*math.sin(alpha_1) + math.sin(alpha_2))*mp*g
    # L = math.sqrt(T2**2 + (mf*g)**2 + 2*mf*g*T2*math.sin(alpha_2))
    # gamma = math.asin(T2*math.cos(alpha_2)/L)

    # command ---> force mapping
    # g / 2125 * omega - 14.937 * g = Force
    
    #roll_base = -gamma * 180.0 / math.pi
    #thrust_base = int((L + 14.937*g)*2125/g) + 2000
    
    # thrust and roll set for the follower
    # thrust_base = 38100
    # roll_base = -3.5
    
    # set the thrust and roll base for the follower here ----------------------------
    
    # when payload is 78 gram
    # thrust_base = 38700
    # roll_base = -4.5

    # when the payload is 113 gram
    thrust_base = 42000
    roll_base = -5.5

    # when payload is 150 gram
    # thrust_base = 43100
    # roll_base = -5.5

    pitch_base = -0.45

    # --------------------------------------------------------------------------------
    
    thrust_count = 0
    roll_count = 0

    thrust_inc_flag = False
    thrust_dec_flag = False
    roll_inc_flag = False
    roll_dec_flag = False

    axis_act_thre = 0.97
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
        lg_stab_leader.add_variable('stabilizer.int_thrust', 'uint16_t')
        height_estimated = EKF_Estimator(sample_time)

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

        Z_feedback_leader_previous = 0.0
        with SyncLogger(scf, lg_stab_leader) as logger:
            # while not flight_terminate_flag:
            for log_entry_leader in logger:
                # print the data from the logging list
                # timestamp = log_entry[0]
                # when acc_z is positive, meaning it points downwards
                # log_acc_z = (log_entry[1].get('acc.z') - 1)*9.81
                # log_thrust = log_entry[1].get('stabilizer.thrust')
                # log_roll = log_entry[1].get('stabilizer.roll')
                # logconf_name = log_entry[2]

                time_index = time_index + 1
                abs_time_delayed = abs_time
                abs_time = time.time() - start_time
                # data = receiver.get_data_sync()
                time.sleep(sample_time)
                data = receiver.get_data()

                data_optical_flow = log_entry_leader[1]

                # digid body ID of the leader from the MoCap
                dp_leader.step(data[(rigid_body_ID_leader-1)*14 : rigid_body_ID_leader*14])
                joystick.step()

                # get the feedback value of the leader
                yaw_feedback_leader = math.atan2(2 * (dp_leader.QW * dp_leader.QZ + dp_leader.QX * dp_leader.QY), 1 - 2 * (dp_leader.QZ * dp_leader.QZ + dp_leader.QY * dp_leader.QY))
                X_feedback_leader = Filter_x.filter(dp_leader.X)
                Y_feedback_leader = Filter_y.filter(dp_leader.Y)
                Z_feedback_leader = Filter_z.filter(dp_leader.Z)
                

                vel_z_feedback_leader = (Z_feedback_leader - Z_feedback_leader_previous)/sample_time
                Z_feedback_leader_previous = Z_feedback_leader

                # the position control of the leader is activated when axis_lt is pushed and axis_rt is released
                if joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() < axis_deact_thre:
                    desired_x_leader = desired_x_leader + joystick.get_axis_ly() * sample_time * 0.5
                    desired_y_leader = desired_y_leader + joystick.get_axis_lx() * sample_time * 0.5
                    desired_z_leader = desired_z_leader - joystick.get_axis_ry() * sample_time * 0.4
                    
                    #---
                    pos_z.des = desired_z_leader
                    #---

                    if yaw_constant_flag_leader:
                        desired_yaw_leader = 0.0
                    else:
                        desired_yaw_leader = Filter_pad_rx.filter(desired_yaw_leader + joystick.get_axis_rx() * sample_time * 20)
                
                # update the reference and get the controller output for the leader
                position_controller_leader.update_reference(desired_x_leader, desired_y_leader, desired_z_leader)

                u_x_leader, u_y_leader, u_z_leader = position_controller_leader.update_error(X_feedback_leader, Y_feedback_leader, Z_feedback_leader)

                # mapping the roll, pithch, yaw and thrust for the leader
                pitch_leader = -(u_x_leader * math.cos(yaw_feedback_leader) + u_y_leader * math.sin(yaw_feedback_leader)) / (math.cos(yaw_feedback_leader) * math.cos(yaw_feedback_leader) 
                          + math.sin(yaw_feedback_leader) * math.sin(yaw_feedback_leader))
                roll_leader = (u_y_leader * math.cos(yaw_feedback_leader) - u_x_leader * math.sin(yaw_feedback_leader)) / (math.cos(yaw_feedback_leader) * math.cos(yaw_feedback_leader) 
                        + math.sin(yaw_feedback_leader) * math.sin(yaw_feedback_leader))
                thrust_leader = limit_thrust(u_z_leader)

                Yaw_error_leader = limit_angular(limit_angular(desired_yaw_leader) - limit_angular(yaw_feedback_leader))
                yaw_leader = - Yaw_error_leader * 50
                
                #print the current position of the quadrotor to check
                # if time_index % 100 == 0:
                #     print(time_index/100,'sec,', ' leader position is', X_feedback_leader, Y_feedback_leader, Z_feedback_leader)

                # send control command to the leader
                if joystick.get_button_l():
                    take_off_flag_leader = True

                if not take_off_flag_leader:
                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 5000)
                else:
                    # --- check the optical flow
                    az_imu = (data_optical_flow['acc.y']*9.81*math.sin(math.radians(data_optical_flow['stateEstimate.roll'])) + data_optical_flow['acc.z']*9.81*math.cos(math.radians(data_optical_flow['stateEstimate.roll'])) - 9.81)
                    pos_z.fdk, vel_z.fdk = height_estimated.EstimatorZ(az_imu,float(data_optical_flow['stateEstimateZ.vz'])/1000,float(data_optical_flow['stateEstimateZ.z'])/1000)
                    opticalz = pos_z.fdk
                    opticalvz = vel_z.fdk
                    pos_z.fdk = Z_feedback_leader
                    vel_z.fdk = vel_z_feedback_leader

                    # update the error
                    pos_z.err = pos_z.des - pos_z.fdk
                    vel_z.err = 0 - vel_z.fdk

                    # pos_z.err = pos_z.des - Z_feedback_leader
                    # vel_z.err = 0 - vel_z_feedback_leader

                    vel_x.fdk = float(data_optical_flow['stateEstimateZ.vx'])/1000
                    vel_y.fdk = float(data_optical_flow['stateEstimateZ.vy'])/1000

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
                    # -----------------------------------------
                    # scf.cf.commander.send_setpoint(roll_leader, pitch_leader, yaw_leader, thrust_leader)
                    scf.cf.commander.send_setpoint(roll_leader, pitch_leader, yaw_leader, thrust_cmd_leader)

                    saver_leader_check.add_elements(
                        pos_z.des,
                        Z_feedback_leader,
                        # pos_z.fdk,
                        opticalz,
                        vel_z_feedback_leader,
                        # vel_z.fdk
                        opticalvz,
                        abs_time
                    )

                
                # push axis_lt and x to land the leader
                if joystick.get_button_x() and joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() < axis_deact_thre :
                    scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                    time.sleep(0.2)
                    scf.cf.commander.send_setpoint(0, 0, 0, 0)
                    break
    # follower flight control branch            
    elif control_which_flag[0] == 1.0:
        lg_stab_follower_1 = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
        lg_stab_follower_1.add_variable('stabilizer.thrust', 'float')
        lg_stab_follower_1.add_variable('stabilizer.roll', 'float')
        lg_stab_follower_1.add_variable('externalforce.T', 'float')
        lg_stab_follower_1.add_variable('stabilizer.pitch', 'float')
        with SyncLogger(scf, lg_stab_follower_1) as logger:
            # while not flight_terminate_flag:
            for log_entry in logger:
                # print the data from the logging list
                timestamp = log_entry[0]
                # when acc_z is positive, meaning it points downwards
                # log_acc_z = (log_entry[1].get('acc.z') - 1)*9.81
                # log_thrust = log_entry[1].get('
                # .thrust')
                # log_roll = log_entry[1].get('stabilizer.roll')
                # logconf_name = log_entry[2]

                time_index = time_index + 1
                abs_time_delayed = abs_time
                abs_time = time.time() - start_time
                # data = receiver.get_data_sync()
                time.sleep(sample_time)
                data = receiver.get_data()

                # digid body ID of the leader from the MoCap
                dp_follower.step(data[(rigid_body_ID_follower_1-1)*14 : rigid_body_ID_follower_1*14])
                joystick.step()

                # get the feedback value of the leader
                yaw_feedback_follower = math.atan2(2 * (dp_follower.QW * dp_follower.QZ + dp_follower.QX * dp_follower.QY), 1 - 2 * (dp_follower.QZ * dp_follower.QZ + dp_follower.QY * dp_follower.QY))
                X_feedback_follower = Filter_x.filter(dp_follower.X)
                Y_feedback_follower = Filter_y.filter(dp_follower.Y)
                Z_feedback_follower = Filter_z.filter(dp_follower.Z)

                # the position control of the follower is activated when both axis_lt and axis_rt are released
                if joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() < axis_deact_thre:
                    desired_x_follower = desired_x_follower + joystick.get_axis_ly() * sample_time * 0.5
                    desired_y_follower = desired_y_follower + joystick.get_axis_lx() * sample_time * 0.5
                    desired_z_follower = desired_z_follower - joystick.get_axis_ry() * sample_time * 0.4
                    if yaw_constant_flag_follower:
                        desired_yaw_follower = 0.0
                    else:
                        desired_yaw_follower = Filter_pad_rx.filter(desired_yaw_follower + joystick.get_axis_rx() * sample_time * 20)
                
                # update the reference and get the controller output for the follower 1
                position_controller_follower.update_reference(desired_x_follower, desired_y_follower, desired_z_follower)

                u_x_follower, u_y_follower, u_z_follower = position_controller_follower.update_error(X_feedback_follower, Y_feedback_follower, Z_feedback_follower)

                # mapping the roll, pithch, yaw and thrust for the leader
                pitch_follower = -(u_x_follower * math.cos(yaw_feedback_follower) + u_y_follower * math.sin(yaw_feedback_follower)) / (math.cos(yaw_feedback_follower) * math.cos(yaw_feedback_follower) 
                          + math.sin(yaw_feedback_follower) * math.sin(yaw_feedback_follower))
                roll_follower = (u_y_follower * math.cos(yaw_feedback_follower) - u_x_follower * math.sin(yaw_feedback_follower)) / (math.cos(yaw_feedback_follower) * math.cos(yaw_feedback_follower) 
                        + math.sin(yaw_feedback_follower) * math.sin(yaw_feedback_follower))
                thrust_follower = limit_thrust(u_z_follower)

                Yaw_error_follower = limit_angular(limit_angular(desired_yaw_follower) - limit_angular(yaw_feedback_follower))
                yaw_follower = - Yaw_error_follower * 50

                # to adjust the roll and thrust commands by the joystick when the position control is off
                thrust_inc_flag_prev = thrust_inc_flag
                thrust_dec_flag_prev = thrust_dec_flag
                roll_inc_flag_prev = roll_inc_flag
                roll_dec_flag_prev = roll_dec_flag

                # rb -> increase the thrust
                if joystick.get_button_rb() and joystick.get_button_lb() < axis_deact_thre:
                    thrust_inc_flag = True
                    if (not thrust_inc_flag_prev) and  thrust_inc_flag:
                        thrust_count = int(thrust_count + 1)
                else:
                    thrust_inc_flag = False 

                # lb -> decrease the thrust
                if joystick.get_button_lb() and joystick.get_button_lb() < axis_deact_thre:
                    thrust_dec_flag = True
                    if (not thrust_dec_flag_prev) and  thrust_dec_flag:
                        thrust_count = int(thrust_count - 1)
                else:
                    thrust_dec_flag = False    

                # b -> increase the roll
                if joystick.get_button_b() and joystick.get_button_lb() < axis_deact_thre:
                    roll_inc_flag = True
                    if (not roll_inc_flag_prev) and  roll_inc_flag:
                        roll_count = int(roll_count + 1)
                else:
                    roll_inc_flag = False 

                # a -> decrease the roll
                if joystick.get_button_a() and joystick.get_button_lb() < axis_deact_thre:
                    roll_dec_flag = True
                    if (not roll_dec_flag_prev) and  roll_dec_flag:
                        roll_count = int(roll_count - 1)
                else:
                    roll_dec_flag = False

                # send control command to the follower
                if joystick.get_button_r():
                    take_off_flag_follower = True
                    
                if not take_off_flag_follower:
                    # follower does not take off
                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 5000)
                else:
                    # follower take off
                    # protection off
                    # if joystick.get_button_y() and joystick.get_button_lb() < axis_deact_thre:
                    if joystick.get_button_y():
                        # position control off
                        roll_cmd = roll_base + roll_count * 0.1
                        thrust_cmd = int(thrust_base + thrust_count * 100)

                        scf.cf.commander.send_setpoint(roll_cmd, pitch_base, 0, thrust_cmd)
                        # if time_index % 100 == 0:
                        #     print('position_control_off', time_index/100, 'sec', ' ,thrust_cmd=', thrust_cmd,
                        #     ' ,roll_cmd=', roll_cmd)

                    else:
                        # position control on
                        scf.cf.commander.send_setpoint(roll_follower, pitch_follower, yaw_follower, thrust_follower)
                
                # print the current position of the quadrotor to check
                # if time_index % 100 == 0:
                #     # print(time_index/100, 'sec,', ' follower position is', X_feedback_follower, Y_feedback_follower, Z_feedback_follower)
                #     print(time_index/100, 'roll_count =', roll_count, ' thrust_count=',thrust_count)

                # push x to land the follower
                if joystick.get_button_x() and joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() < axis_deact_thre :
                    scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                    time.sleep(0.2)
                    scf.cf.commander.send_setpoint(0, 0, 0, 0)
                    break
        

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    #controlQueues = [Queue() for _ in range(len(uris))]
    saver = savemat.DataSaver('Abs_time', 'X', 'Y', 'Z', 'QW', 'QX', 'QY', 'QZ', 
                              'roll', 'pitch', 'yaw', 'thrust',
                              'angleY')

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

    saver_leader_check = savemat.DataSaver(
        'pos_z_des',
        'pos_z_fdk_mocap',
        'pos_z_fdk_optical',
        'vel_z_fdk_mocap',
        'vel_z_fdk_optical',
        'abs_time'
    )    
    # starts the swarm!!!
    with Swarm(URIS_swarm, factory=factory) as swarm:
        swarm.parallel_safe(crazyflie_control, args_dict = control_flag_args)

    saver_leader_check.save2mat('DATA_mocap_optical_check/')
    
    receiver.stop_thread()
    # saver.save2mat('DataExchange/')
    joystick.quit()

