"""
This is the program mainly for the outdoor experiment for one leader and one follower, where
-> the leader uses optical flow
-> the follower use passive stability and force moded referenced control


!----- NOV 1, 2021
but the follower still uses the logged data as feedback from the robot
not the online thrust controller
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

    time_index = 0

    follower_time_index = 0.0
    
    # --------------------------------------------------------------------------------
    # the threshold of reading the active signal from the joystick
    axis_act_thre = 0.95
    axis_deact_thre = 0.05

    # ------- leader control branch -------
    if control_which_flag[0] == 0.0:
            # the logging data list, for the leader
        lg_stab_leader = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
        lg_stab_leader.add_variable('acc.z', 'float')
        lg_stab_leader.add_variable('stabilizer.thrust', 'float')
        lg_stab_leader.add_variable('stabilizer.roll', 'float')

        leader_max_vel = 0.7

        with SyncLogger(scf, lg_stab_leader) as logger_leader:
            # while not flight_terminate_flag:
            for log_entry in logger_leader:
                # print the data from the logging list

                time_index = time_index + 1
                abs_time = time.time() - start_time
                # data = receiver.get_data_sync()
                time.sleep(sample_time)

                # digid body ID of the leader from the MoCap
                joystick.step()

                # the position control of the leader is activated when axis_lt is pushed and axis_rt is released
                if joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() < axis_deact_thre:

                    leader.vel_x.des = -joystick.get_axis_ly() * leader_max_vel
                    leader.vel_y.des = -joystick.get_axis_lx() * leader_max_vel
                    leader.yaw_rate.des = joystick.get_axis_rx() * 30
                    leader.pos_z.des = leader.pos_z.des - joystick.get_axis_ry() * sample_time * 0.4             
            
                # send control command to the leader
                if joystick.get_button_l():
                    leader.take_off_flag = True

                # push the lt and rt simutaneously to land the two robots
                if joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() > axis_act_thre:
                    leader.land_flag = True

                if not leader.take_off_flag:
                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 5000)
                    # push axis_lt and x to land the leader
                    if joystick.get_button_x() and joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() < axis_deact_thre:
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break
                
                if leader.take_off_flag and not leader.land_flag:
                    # scf.cf.commander.send_position_setpoint(desired_x_leader, desired_y_leader, 
                    # desired_z_leader, leader.yaw_rate.des)
                    scf.cf.commander.send_hover_setpoint(leader.vel_x.des, leader.vel_y.des, 
                    leader.yaw_rate.des, leader.pos_z.des)

                    # push x to terminate the flight
                    if joystick.get_button_x() and joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() < axis_deact_thre :
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break

                if leader.land_flag:
                    scf.cf.commander.send_setpoint(2.0, 0, 0, 35000)
                    # push axis_lt and x to land the leader
                    if joystick.get_button_x() and joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() < axis_deact_thre :
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break


    # follower 1 flight control branch            
    elif control_which_flag[0] == 1.0:
         # the logging date list, for the follower 1
        lg_stab_follower_1 = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
        lg_stab_follower_1.add_variable('stabilizer.thrust', 'float')
        lg_stab_follower_1.add_variable('stabilizer.roll', 'float')
        lg_stab_follower_1.add_variable('externalforce.T', 'float')
        lg_stab_follower_1.add_variable('stabilizer.pitch', 'float')

        # force controller parameters set
        # thrust_I_gain = 25
        thrust_I_gain = 25
        thrust_P_gain = 50
        # the feedforward term should be paid special attention
        feedforward_term = 29000

        thrust_err_limit = 0.15

        thrust_base = 25000
        thrust_cmd_I = 12000
        thrust_cmd = thrust_base # -> initial value for iteration

        # the value is: thrust = 2.8575 N, gamma = -7.1735 deg
        desired_thrust = 2.85
        desired_roll = -7.1
        
        with SyncLogger(scf, lg_stab_follower_1) as logger_follower_1:
            # while not flight_terminate_flag:
            for log_entry in logger_follower_1:
                # print the data from the logging list

                abs_time = time.time() - start_time
                time.sleep(sample_time)

                joystick.step()

                # send control command to the follower
                if joystick.get_button_r():
                    follower_1.take_off_flag = True

                if joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() > axis_act_thre:
                    follower_1.land_flag = True
                    
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

                        if thrust_cmd <= 33000:
                            thrust_cmd = int(33000)

                        # this is to prevent too much force in the taking off stage
                        if follower_time_index <= 1000:
                            if thrust_cmd >= 43000:
                                thrust_cmd = int(43000)
                        
                        scf.cf.commander.send_setpoint(desired_roll, 0, 0, thrust_cmd)
                        
                        
                        # only log when the position control is off
                        saver_follower_1.add_elements(log_entry[1].get('stabilizer.roll'), 
                            log_entry[1].get('stabilizer.pitch'),
                            log_entry[1].get('externalforce.T'),
                            thrust_cmd
                            )

                    # terminate the flight
                    if joystick.get_button_x() and joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() < axis_deact_thre :
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break

                    follower_time_index = follower_time_index + 1

                
                if follower_1.land_flag:
                    scf.cf.commander.send_setpoint(-5, 0, 0, 35000)
                    if joystick.get_button_x() and joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() < axis_deact_thre :
                        scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                        time.sleep(0.2)
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        break
        

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # set the sample here, hz
    sample_rate = 100
    sample_time = 1 / sample_rate
    sample_rate_in_ms = int(sample_time * 1000)

    # creat agents
    leader = Creat_Agent()
    follower_1 = Creat_Agent()

    leader.pos_z.des = 0.15

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

    factory = CachedCfFactory(rw_cache='./cache')
    
    saver_follower_1 = savemat.DataSaver('roll_actual', 'pitch_actual', 'thrust_actual', 
         'thrust_cmd')


    # starts the swarm!!!
    with Swarm(URIS_swarm, factory=factory) as swarm:
        swarm.parallel_safe(crazyflie_control, args_dict = control_flag_args)
    
    # saver_follower_1.save2mat('DATA_leader_optical_flow/')

    joystick.quit()

