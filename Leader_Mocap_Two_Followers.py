"""
this is the program for the passive stability formation
with TWO followers and another robot as the leader 

!!!! ---- CAUTION ---- !!!! 
the firmware of the followers are modified that the 
sendsetpoint gives the thrust in NEWTON directly

the version 1.0 is finished by Estel in Sep 11, 2021


----!!!
modified at NOV 1, 2021
the joystick button are re-mapped
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

    axis_act_thre = 0.95
    axis_deact_thre = 0.05

    coe_newton_to_cmd = 10000/4

    # # the deisired thrust and attitude for the FOLLOWER ONE
    follower_1.thrust_inNewton.des = 2.67
    follower_1.roll.des = -4.5
    follower_1.pitch.des = 7.7

    # # the desired thrust and attitude for the FOLLOWER TWO
    follower_2.thrust_inNewton.des = 2.67
    follower_2.roll.des = -4.5
    follower_2.pitch.des = -7.7

    # # # the deisired thrust and attitude for the FOLLOWER ONE
    # follower_1.thrust_inNewton.des = 3.04
    # follower_1.roll.des = -7.0
    # follower_1.pitch.des = 12.0

    # # # the desired thrust and attitude for the FOLLOWER TWO
    # follower_2.thrust_inNewton.des = 3.04
    # follower_2.roll.des = -7.0
    # follower_2.pitch.des = -12.0

    # the deisired thrust and attitude for the FOLLOWER ONE
    # follower_1.thrust_inNewton.des = 1.8
    # follower_1.roll.des = 0.0
    # follower_1.pitch.des = 0.0

    # the desired thrust and attitude for the FOLLOWER TWO
    # follower_2.thrust_inNewton.des = 1.8
    # follower_2.roll.des = 0.0
    # follower_2.pitch.des = 0.0

    # data processor, filtering the data from MoCap and command from joystick
    dp_leader = GeneralFcn.RealTimeProcessor()
    # set the rigidbody ID
    rigid_body_ID_leader = 1

    Filter_x = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)
    Filter_y = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)
    Filter_z = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)

    Filter_pad_ly = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=65, fs=sample_rate)
    Filter_pad_lx = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=65, fs=sample_rate)
    Filter_pad_ry = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=65, fs=sample_rate)
    Filter_pad_rx = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=50, fs=sample_rate)

    position_controller_leader = PID_ControllerThreeAixs(sample_time,
                                                  10, 1, 20, 0,
                                                  7, 1, 20, 0,
                                                  10000, 500, 10000, 40000, )

    yaw_constant_flag_leader = True

    # ------- LEADER CONTROL BRANCH -------
    if control_which_flag[0] == 0.0:
        # the logging data list, for the leader
        lg_stab_leader = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
        lg_stab_leader.add_variable('acc.z', 'float')
        lg_stab_leader.add_variable('stabilizer.thrust', 'float')
        lg_stab_leader.add_variable('stabilizer.roll', 'float')

        with SyncLogger(scf, lg_stab_leader) as logger_leader:
            # while not flight_terminate_flag:
            for log_entry in logger_leader:
                
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


    # ------- FOLLOWER 1 CONTROL BRANCH -------
    elif control_which_flag[0] == 1.0:

        # follower_1.thrust_inNewton.des = 2.65
        # follower_1.roll.des = -2.6
        # follower_1.pitch.des = 4.46

        # the logging data list, for the leader
        lg_stab_follower_1 = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
        lg_stab_follower_1.add_variable('stabilizer.thrust', 'float')
        # lg_stab_follower_1.add_variable('stabilizer.roll', 'float')

        with SyncLogger(scf, lg_stab_follower_1) as logger_followrer_1:
            # while not flight_terminate_flag:
            for log_entry in logger_followrer_1:
                
                time.sleep(sample_time)
                joystick.step()

                # send control command to the leader
                if joystick.get_button_r() and joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() < axis_act_thre:
                    follower_1.take_off_flag = True
                    follower_1.event_flag = int(1)
                    follower_1.take_off_time = time.time()

                # land the formation when the LT and RT are pushed
                if joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() > axis_act_thre:
                    follower_1.land_flag = True
                    follower_1.event_flag = int(2)
                    follower_1.land_time = time.time()

                if not follower_1.take_off_flag:
                    scf.cf.commander.send_setpoint(0, 0, 0, 600)
                    time.sleep(0.1)
                    
                    if joystick.get_button_x():
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break
                
                if follower_1.take_off_flag and not follower_1.land_flag:
                    if (time.time() - follower_1.take_off_time) <= 6:
                        time_scale_follower_1 = (time.time() - follower_1.take_off_time)/6
                        desired_thrust_inNewton_temp_follower_1 = time_scale_follower_1*0.9*follower_1.thrust_inNewton.des
                        desired_roll_temp_follower_1 = time_scale_follower_1*0.8
                        desired_pitch_temp_follower_1 = time_scale_follower_1*0.8                      
                        scf.cf.commander.send_setpoint(desired_roll_temp_follower_1, desired_pitch_temp_follower_1, 0, int(desired_thrust_inNewton_temp_follower_1*coe_newton_to_cmd))
                   
                    elif (time.time() - follower_1.take_off_time) <= 8:                       
                        scf.cf.commander.send_setpoint(follower_1.roll.des*0.8, follower_1.pitch.des*0.8, 0, int(0.8*follower_1.thrust_inNewton.des*coe_newton_to_cmd))
                    
                    elif (time.time() - follower_1.take_off_time) <= 10:                       
                        scf.cf.commander.send_setpoint(follower_1.roll.des*0.9, follower_1.pitch.des*0.9, 0, int(0.9*follower_1.thrust_inNewton.des*coe_newton_to_cmd))
                    
                    else:  
                        scf.cf.commander.send_setpoint(follower_1.roll.des, follower_1.pitch.des, 0, int(follower_1.thrust_inNewton.des * coe_newton_to_cmd))

                    if joystick.get_button_x():
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break

                if follower_1.land_flag:
                    if (time.time() - follower_1.land_time) <= 7:
                        time_scale_follower_1 = 1 - (time.time() - follower_1.land_time)/7

                        if time_scale_follower_1 <= 0.05:
                            time_scale_follower_1 = 0.05

                        desired_thrust_inNewton_temp_follower_1 = time_scale_follower_1*follower_1.thrust_inNewton.des*0.9
                        desired_roll_temp_follower_1 = time_scale_follower_1*follower_1.roll.des
                        desired_pitch_temp_follower_1 = time_scale_follower_1*follower_1.pitch.des

                        scf.cf.commander.send_setpoint(desired_roll_temp_follower_1, desired_pitch_temp_follower_1, 0, int(desired_thrust_inNewton_temp_follower_1 * coe_newton_to_cmd))
                    
                    else:
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)

                    if joystick.get_button_x():
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break


    # ------- FOLLOWER TWO CONTROL BRANCH -------         
    elif control_which_flag[0] == 2.0:
        # follower_2.thrust_inNewton.des = 2.65
        # follower_2.roll.des = -2.6
        # follower_2.pitch.des = -4.46

        # the logging data list, for the leader
        lg_stab_follower_2 = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
        lg_stab_follower_2.add_variable('stabilizer.thrust', 'float')
        # lg_stab_follower_2.add_variable('stabilizer.roll', 'float')

        with SyncLogger(scf, lg_stab_follower_2) as logger_followrer_2:
            # while not flight_terminate_flag:
            for log_entry in logger_followrer_2:
                
                time.sleep(sample_time)
                joystick.step()

                # send control command to the leader
                if joystick.get_button_r() and joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() > axis_act_thre:
                    follower_2.take_off_flag = True
                    follower_2.event_flag = int(1)
                    follower_2.take_off_time = time.time()

                # land the formation when the LT and RT are pushed
                if joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() > axis_act_thre:
                    follower_2.land_flag = True
                    follower_2.event_flag = int(2)
                    follower_2.land_time = time.time()

                if not follower_2.take_off_flag:
                    scf.cf.commander.send_setpoint(0, 0, 0, 600)
                    time.sleep(0.1)
                    
                    if joystick.get_button_x():
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break
                
                if follower_2.take_off_flag and not follower_2.land_flag:
                    if (time.time() - follower_2.take_off_time) <= 6:
                        time_scale_follower_2 = (time.time() - follower_2.take_off_time)/6
                        desired_thrust_inNewton_temp_follower_2 = time_scale_follower_2*0.8*follower_2.thrust_inNewton.des
                        desired_roll_temp_follower_2 = time_scale_follower_2*0.8
                        desired_pitch_temp_follower_2 = time_scale_follower_2*0.8                   
                        scf.cf.commander.send_setpoint(desired_roll_temp_follower_2, desired_pitch_temp_follower_2, 0, int(desired_thrust_inNewton_temp_follower_2*coe_newton_to_cmd))
                   
                    elif (time.time() - follower_2.take_off_time) <= 8:                       
                        scf.cf.commander.send_setpoint(follower_2.roll.des*0.9, follower_2.pitch.des*0.9, 0, int(0.9*follower_2.thrust_inNewton.des*coe_newton_to_cmd))
                    
                    elif (time.time() - follower_2.take_off_time) <= 10:                       
                        scf.cf.commander.send_setpoint(follower_2.roll.des*0.95, follower_2.pitch.des*0.95, 0, int(0.95*follower_2.thrust_inNewton.des*coe_newton_to_cmd))
                    
                    else:  
                        scf.cf.commander.send_setpoint(follower_2.roll.des, follower_2.pitch.des, 0, int(follower_2.thrust_inNewton.des * coe_newton_to_cmd))

                    if joystick.get_button_x():
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break

                if follower_2.land_flag:
                    if (time.time() - follower_2.land_time) <= 7:
                        time_scale_follower_2 = 1 - (time.time() - follower_2.land_time)/7

                        if time_scale_follower_2 <= 0.05:
                            time_scale_follower_2 = 0.05

                        desired_thrust_inNewton_temp_follower_2 = time_scale_follower_2*follower_2.thrust_inNewton.des*0.9
                        desired_roll_temp_follower_2 = time_scale_follower_2*follower_2.roll.des
                        desired_pitch_temp_follower_2 = time_scale_follower_2*follower_2.pitch.des

                        scf.cf.commander.send_setpoint(desired_roll_temp_follower_2, desired_pitch_temp_follower_2, 0, int(desired_thrust_inNewton_temp_follower_2 * coe_newton_to_cmd))
                    
                    else:
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)

                    if joystick.get_button_x():
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break
        

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    
    receiver = UdpReceiver.UdpRigidBodies()

    # set the sample here, hz
    sample_rate = 100
    sample_time = 1 / sample_rate
    sample_rate_in_ms = int(sample_time * 1000)
    # receiver.start_thread()
    receiver.start_thread()


    # creat agents
    # leader = Creat_Agent()
    leader = Creat_Agent()
    follower_1 = Creat_Agent()
    follower_2 = Creat_Agent()
    # payload = Creat_Agent()

    # set the initial position of the agents
    leader.pos_x.des = 0.2
    leader.pos_y.des = 0.55
    leader.pos_z.des = 0.15

    uri_leader = 'radio://0/31/2M'
    uri_follower_1 = 'radio://0/101/2M'  
    uri_follower_2 = 'radio://0/66/2M'

    control_flag_args = {
        uri_leader : [[0.0, 0.0, 0.0]],
        uri_follower_1: [[1.0, 1.0, 1.0]],
        uri_follower_2: [[2.0, 2.0, 2.0]]
    }

    URIS_swarm = {uri_leader, uri_follower_1, uri_follower_2}


    factory = CachedCfFactory(rw_cache='./cache')
    
    # starts the swarm!!!
    with Swarm(URIS_swarm, factory=factory) as swarm:
        swarm.parallel_safe(crazyflie_control, args_dict = control_flag_args)
    
    receiver.stop_thread()

    joystick.quit()

