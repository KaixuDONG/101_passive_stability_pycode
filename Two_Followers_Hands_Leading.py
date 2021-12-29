"""
this is the program for the passive stability formation
with TWO followers and the hands holding as the leader

!!!! ---- CAUTION ---- !!!! 
the firmware of the followers are modified that the 
sendsetpoint gives the thrust in NEWTON directly

the version 1.0 is finished by Estel in Sep 11, 2021
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
    aixs_deact_thre = 0.05

    # since there are four motors
    coe_newton_to_cmd = 10000/4

    # # the deisired thrust and attitude for the FOLLOWER ONE
    follower_1.thrust_inNewton.des = 2.65
    follower_1.roll.des = -3
    follower_1.pitch.des = 4.5

    # # the desired thrust and attitude for the FOLLOWER TWO
    follower_2.thrust_inNewton.des = 2.65
    follower_2.roll.des = -3
    follower_2.pitch.des = -4.5

    # the deisired thrust and attitude for the FOLLOWER ONE
    # follower_1.thrust_inNewton.des = 1.8
    # follower_1.roll.des = 0.0
    # follower_1.pitch.des = 0.0

    # the desired thrust and attitude for the FOLLOWER TWO
    # follower_2.thrust_inNewton.des = 1.8
    # follower_2.roll.des = 0.0
    # follower_2.pitch.des = 0.0

    # ------- FOLLOWER 1 CONTROL BRANCH -------
    if control_which_flag[0] == 1.0:

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
                if joystick.get_button_l():
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
                if joystick.get_button_r():
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

    # set the sample here, hz
    sample_rate = 100
    sample_time = 1 / sample_rate
    sample_rate_in_ms = int(sample_time * 1000)
    # receiver.start_thread()

    # creat agents
    # leader = Creat_Agent()
    follower_1 = Creat_Agent()
    follower_2 = Creat_Agent()
    # payload = Creat_Agent()

    uri_follower_1 = 'radio://0/101/2M'  
    uri_follower_2 = 'radio://0/66/2M'

    control_flag_args = {
        uri_follower_1: [[1.0, 1.0, 1.0]],
        uri_follower_2: [[2.0, 2.0, 2.0]]
    }

    URIS_swarm = {uri_follower_1, uri_follower_2}


    factory = CachedCfFactory(rw_cache='./cache')
    
    # starts the swarm!!!
    with Swarm(URIS_swarm, factory=factory) as swarm:
        swarm.parallel_safe(crazyflie_control, args_dict = control_flag_args)
    

    joystick.quit()

