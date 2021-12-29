"""
this is the program is for the swarm with THREE robots
-> all the robots use Mocap
-> need to check whether to record the states of the payload


NOV 1, 2021
double checked
"""

# release the LT and RT to control the leader
# push the LT and release RT to control the follower 1
# release the LT and push RT to control the follower 2
# push the LT and RT to land the formation

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
from cooperative_transport.controllers import Creat_Agent, PID_ControllerThreeAixs
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

    # data processor, filtering the data from MoCap and command from joystick
    dp_leader = GeneralFcn.RealTimeProcessor()
    dp_follower_1 = GeneralFcn.RealTimeProcessor()
    dp_follower_2 = GeneralFcn.RealTimeProcessor()


    # set the rigidbody ID
    rigid_body_ID_leader = 1
    rigid_body_ID_follower_1 = 2
    rigid_body_ID_follower_2 = 3

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

    position_controller_follower_1 =  PID_ControllerThreeAixs(sample_time,
                                                  10, 1, 20, 0,
                                                  7, 1, 20, 0,
                                                  10000, 500, 10000, 40000, ) 

    position_controller_follower_2 =  PID_ControllerThreeAixs(sample_time,
                                                  10, 1, 20, 0,
                                                  7, 1, 20, 0,
                                                  10000, 500, 10000, 40000, ) 

    # if yaw_constant_flag is Ture, then the desired yaw will set to be 0 for the whole flight
    yaw_constant_flag_leader = True
    yaw_constant_flag_follower_1 = True
    yaw_constant_flag_follower_2 = True
    
    # take off flag for the leader and follower
    # leader.take_off_flag = False
    # follower_1.take_off_flag = False
    # follower_2.take_off_flag = False

  
    axis_act_thre = 0.95
    axis_deact_thre = 0.05

    # ------- leader control branch -------
    if control_which_flag[0] == 0.0:
        # the logging data list, for the leader
        lg_stab_leader = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
        lg_stab_leader.add_variable('stabilizer.thrust', 'float')

        with SyncLogger(scf, lg_stab_leader) as logger:
            # while not flight_terminate_flag:
            for log_entry in logger:
                # print the data from the logging list
 
                time_index = time_index + 1
                abs_time_delayed = abs_time
                abs_time = time.time() - start_time
                
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
                leader.yaw.ctrl_cmd = - leader.yaw.err * 50
                

                # send control command to the leader
                if joystick.get_button_l() and joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() < axis_deact_thre:
                    leader.take_off_flag = True

                if not leader.take_off_flag:
                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 5000)
                    # push x to land the leader with LT and RT released
                    if joystick.get_button_x() and joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() < axis_deact_thre :
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break
                else:
                    scf.cf.commander.send_setpoint(leader.roll.ctrl_cmd, leader.pitch.ctrl_cmd, leader.yaw.ctrl_cmd, leader.thrus_cmd.ctrl_cmd)
                    # push x to land the leader with LT and RT released
                    if joystick.get_button_x() and joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() < axis_deact_thre :
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break
    
    # follower_1 flight control branch            
    elif control_which_flag[0] == 1.0:
        lg_stab_follower_1 = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
        lg_stab_follower_1.add_variable('stabilizer.thrust', 'float')
        with SyncLogger(scf, lg_stab_follower_1) as logger:
            # while not flight_terminate_flag:
            for log_entry in logger:
                # print the data from the logging list
 
                # time_index = time_index + 1
                # abs_time_delayed = abs_time
                # abs_time = time.time() - start_time
                
                time.sleep(sample_time)
                data = receiver.get_data()

                # digid body ID of the follower_1 from the MoCap
                dp_follower_1.step(data[(rigid_body_ID_follower_1-1)*14 : rigid_body_ID_follower_1*14])
                joystick.step()

                # get the feedback value of the follower_1
                follower_1.yaw.fdk = math.atan2(2 * (dp_follower_1.QW * dp_follower_1.QZ + dp_follower_1.QX * dp_follower_1.QY), 1 - 2 * (dp_follower_1.QZ * dp_follower_1.QZ + dp_follower_1.QY * dp_follower_1.QY))
                follower_1.pos_x.fdk = Filter_x.filter(dp_follower_1.X)
                follower_1.pos_y.fdk = Filter_y.filter(dp_follower_1.Y)
                follower_1.pos_z.fdk = Filter_z.filter(dp_follower_1.Z)

                # the position control of the follower_1 is activated when LT is pushed and RT is released
                if joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() < axis_deact_thre:
                    follower_1.pos_x.des = follower_1.pos_x.des + joystick.get_axis_ly() * sample_time * 0.5
                    follower_1.pos_y.des = follower_1.pos_y.des + joystick.get_axis_lx() * sample_time * 0.5
                    follower_1.pos_z.des = follower_1.pos_z.des - joystick.get_axis_ry() * sample_time * 0.4
                    if yaw_constant_flag_follower_1:
                        follower_1.yaw.des = 0.0
                    else:
                        follower_1.yaw.des = Filter_pad_rx.filter(follower_1.yaw.des + joystick.get_axis_rx() * sample_time * 20)
                
                # update the reference and get the controller output for the follower_1
                position_controller_follower_1.update_reference(follower_1.pos_x.des, follower_1.pos_y.des, follower_1.pos_z.des)

                u_x_follower_1, u_y_follower_1, u_z_follower_1 = position_controller_follower_1.update_error(follower_1.pos_x.fdk, follower_1.pos_y.fdk, follower_1.pos_z.fdk)

                # mapping the roll, pithch, yaw and thrust for the follower_1
                follower_1.pitch.ctrl_cmd = -(u_x_follower_1 * math.cos(follower_1.yaw.fdk) + u_y_follower_1 * math.sin(follower_1.yaw.fdk)) / (math.cos(follower_1.yaw.fdk) * math.cos(follower_1.yaw.fdk) 
                          + math.sin(follower_1.yaw.fdk) * math.sin(follower_1.yaw.fdk))
                follower_1.roll.ctrl_cmd = (u_y_follower_1 * math.cos(follower_1.yaw.fdk) - u_x_follower_1 * math.sin(follower_1.yaw.fdk)) / (math.cos(follower_1.yaw.fdk) * math.cos(follower_1.yaw.fdk) 
                        + math.sin(follower_1.yaw.fdk) * math.sin(follower_1.yaw.fdk))
                follower_1.thrus_cmd.ctrl_cmd = limit_thrust(u_z_follower_1)

                follower_1.yaw.err = limit_angular(limit_angular(follower_1.yaw.des) - limit_angular(follower_1.yaw.fdk))
                follower_1.yaw.ctrl_cmd = - follower_1.yaw.err * 50
                

                # send control command to the follower_1
                if joystick.get_button_r() and joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() < axis_deact_thre:
                    follower_1.take_off_flag = True

                # follower_1 landing
                if not follower_1.take_off_flag:
                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 5000)
                    # push x to land the follower_1 with LT and RT released
                    if joystick.get_button_x() and joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() < axis_deact_thre:
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break
                
                # follower_2 take off
                else:
                    scf.cf.commander.send_setpoint(follower_1.roll.ctrl_cmd, follower_1.pitch.ctrl_cmd, follower_1.yaw.ctrl_cmd, follower_1.thrus_cmd.ctrl_cmd)
                    # push x to land the follower_1 with LT and RT released
                    if joystick.get_button_x() and joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() < axis_deact_thre:
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break

    # follower_2 flight control branch            
    elif control_which_flag[0] == 2.0:
        lg_stab_follower_2 = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
        lg_stab_follower_2.add_variable('stabilizer.thrust', 'float')
        with SyncLogger(scf, lg_stab_follower_2) as logger:
            # while not flight_terminate_flag:
            for log_entry in logger:
                # print the data from the logging list
 
                # time_index = time_index + 1
                # abs_time_delayed = abs_time
                # abs_time = time.time() - start_time
                
                time.sleep(sample_time)
                data = receiver.get_data()

                # digid body ID of the follower_2 from the MoCap
                dp_follower_2.step(data[(rigid_body_ID_follower_2-1)*14 : rigid_body_ID_follower_2*14])
                joystick.step()

                # get the feedback value of the follower_2
                follower_2.yaw.fdk = math.atan2(2 * (dp_follower_2.QW * dp_follower_2.QZ + dp_follower_2.QX * dp_follower_2.QY), 1 - 2 * (dp_follower_2.QZ * dp_follower_2.QZ + dp_follower_2.QY * dp_follower_2.QY))
                follower_2.pos_x.fdk = Filter_x.filter(dp_follower_2.X)
                follower_2.pos_y.fdk = Filter_y.filter(dp_follower_2.Y)
                follower_2.pos_z.fdk = Filter_z.filter(dp_follower_2.Z)

                # the position control of the follower_2 is activated when LT is released and RT is pushed
                if joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() > axis_act_thre:
                    follower_2.pos_x.des = follower_2.pos_x.des + joystick.get_axis_ly() * sample_time * 0.5
                    follower_2.pos_y.des = follower_2.pos_y.des + joystick.get_axis_lx() * sample_time * 0.5
                    follower_2.pos_z.des = follower_2.pos_z.des - joystick.get_axis_ry() * sample_time * 0.4
                    if yaw_constant_flag_follower_2:
                        follower_2.yaw.des = 0.0
                    else:
                        follower_2.yaw.des = Filter_pad_rx.filter(follower_2.yaw.des + joystick.get_axis_rx() * sample_time * 20)
                
                # update the reference and get the controller output for the follower_2
                position_controller_follower_2.update_reference(follower_2.pos_x.des, follower_2.pos_y.des, follower_2.pos_z.des)

                u_x_follower_2, u_y_follower_2, u_z_follower_2 = position_controller_follower_2.update_error(follower_2.pos_x.fdk, follower_2.pos_y.fdk, follower_2.pos_z.fdk)

                # mapping the roll, pithch, yaw and thrust for the follower_2
                follower_2.pitch.ctrl_cmd = -(u_x_follower_2 * math.cos(follower_2.yaw.fdk) + u_y_follower_2 * math.sin(follower_2.yaw.fdk)) / (math.cos(follower_2.yaw.fdk) * math.cos(follower_2.yaw.fdk) 
                          + math.sin(follower_2.yaw.fdk) * math.sin(follower_2.yaw.fdk))
                follower_2.roll.ctrl_cmd = (u_y_follower_2 * math.cos(follower_2.yaw.fdk) - u_x_follower_2 * math.sin(follower_2.yaw.fdk)) / (math.cos(follower_2.yaw.fdk) * math.cos(follower_2.yaw.fdk) 
                        + math.sin(follower_2.yaw.fdk) * math.sin(follower_2.yaw.fdk))
                follower_2.thrus_cmd.ctrl_cmd = limit_thrust(u_z_follower_2)

                follower_2.yaw.err = limit_angular(limit_angular(follower_2.yaw.des) - limit_angular(follower_2.yaw.fdk))
                follower_2.yaw.ctrl_cmd = - follower_2.yaw.err * 50
                

                # send control command to the follower_2
                if joystick.get_button_r() and joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() > axis_act_thre:
                    follower_2.take_off_flag = True

                # follower_2 landing
                if not follower_2.take_off_flag:
                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 5000)
                    # push x to land the follower_2 with LT and RT released
                    if joystick.get_button_x() and joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() > axis_act_thre:
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break
                # follower_2 take off
                else:
                    scf.cf.commander.send_setpoint(follower_2.roll.ctrl_cmd, follower_2.pitch.ctrl_cmd, follower_2.yaw.ctrl_cmd, follower_2.thrus_cmd.ctrl_cmd)
                    # push x to land the follower_2 with LT and RT released
                    if joystick.get_button_x() and joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() > axis_act_thre:
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break


# main control loop, define the initial positions for all the agents
if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # udp receiver
    receiver = UdpReceiver.UdpRigidBodies()
   
   
    # set the sample here, hz
    sample_rate = 100
    sample_time = 1 / sample_rate
    sample_rate_in_ms = int(sample_time * 1000)
    receiver.start_thread()


    # creat agents
    leader = Creat_Agent()
    follower_1 = Creat_Agent()
    follower_2 = Creat_Agent()
    
    # initial states for the leader
    leader.pos_x.des = 0.41
    leader.pos_y.des = 0.63
    leader.pos_z.des = 0.1

    # initial states for the follower_1 
    follower_1.pos_x.des = -0.21
    follower_1.pos_y.des = -0.64
    follower_1.pos_z.des = 0.15

    # initial states for the follower_2 
    follower_2.pos_x.des = 1.13
    follower_2.pos_y.des = -0.43
    follower_2.pos_z.des = 0.15

    # set the URI of the quadrotors which you wish to control in the swarm

    #uri_leader = 'radio://0/23/2M/E7E7E7E701'  
    #uri_follower_1 = 'radio://0/101/2M/E7E7E7E702'
    uri_leader = 'radio://0/31/2M'  
    uri_follower_1 = 'radio://0/101/2M'
    uri_follower_2 = 'radio://0/66/2M'

    control_flag_args = {
        uri_leader: [[0.0, 0.0, 0.0]],
        uri_follower_1: [[1.0, 1.0, 1.0]],
        uri_follower_2: [[2.0, 2.0, 2.0]]
    }

    URIS_swarm = {uri_leader, uri_follower_1, uri_follower_2}

    # logging.basicConfig(level=logging.ERROR)

    factory = CachedCfFactory(rw_cache='./cache')
    
    # saver_follower_1 = savemat.DataSaver('roll_actual', 'pitch_actual', 'thrust_actual', 
    #      'thrust_cmd')


    # starts the swarm!!!
    with Swarm(URIS_swarm, factory=factory) as swarm:
        swarm.parallel_safe(crazyflie_control, args_dict = control_flag_args)
    
    # saver_follower_1.save2mat('DATA_leader_optical_flow/')

    receiver.stop_thread()

    joystick.quit()