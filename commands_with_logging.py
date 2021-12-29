"""
commands_with_logging.py

to send commands from PID controller for position control,
and send constant setpoint to serve as the follower,
log the data from the flight process as well

this program can control only one quadrotor, by the help of MoCap


-> Nov 1, 2021
----------!!!!------
the code is from a very old version

but has basic functionality of controlling a robot by the help of mocap
and with the function of logging data form a robot
"""

import logging
import time
import math
from SaveMAT import savemat
# libs from crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

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

# saturation funciton
def limit_thrust(thrust_cmd):
    if thrust_cmd > 65535:
        thrust_cmd = 65535
    if thrust_cmd < 0:
        thrust_cmd = 0
    return int(thrust_cmd)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    saver = savemat.DataSaver('Abs_time', 'X', 'Y', 'Z', 'QW', 'QX', 'QY', 'QZ', 
                              'roll', 'pitch', 'yaw', 'thrust',
                              'angleY')

    # my_trajectory = trajectory_loader.Trajectory('/CFrobot/DataExchange/Trajecotries/T1.mat')
    # my_trajectory.update()
    
    # udp receiver
    receiver = UdpReceiver.UdpRigidBodies()
    # sample_rate = receiver.get_sample_rate()
    # set the sample here, hz
    sample_rate = 100
    sample_time = 1 / sample_rate
    sample_rate_in_ms = int(sample_time * 1000)
    receiver.start_thread()
    # the logging data list
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
    # lg_stab.add_variable('stabilizer.roll', 'float')
    # lg_stab.add_variable('stabilizer.pitch', 'float')
    # lg_stab.add_variable('stabilizer.yaw', 'float')
    lg_stab.add_variable('acc.z', 'float')
    lg_stab.add_variable('stabilizer.thrust', 'float')
    lg_stab.add_variable('stabilizer.roll', 'float')
    # lg_stab.add_variable('stateEstimate.roll', 'float')
    # lg_stab.add_variable('stateEstimate.pitch', 'float')
    # lg_stab.add_variable('externalforce.T', 'float')


    # data processor, filtering the data from MoCap and command from joystick
    dp = GeneralFcn.RealTimeProcessor()
    Filter_x = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)
    Filter_y = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)
    Filter_z = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)

    Filter_pad_ly = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=65, fs=sample_rate)
    Filter_pad_lx = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=65, fs=sample_rate)
    Filter_pad_ry = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=65, fs=sample_rate)
    Filter_pad_rx = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=50, fs=sample_rate)

    # controller parameter
    trajectory_on = True
    position_controller = PID_ControllerThreeAixs(sample_time,
                                                  10, 1, 20, 0,
                                                  7, 1, 20, 0,
                                                  10000, 500, 10000, 40000, )

    # desired initial position, (in world frame)
    desired_x = 1.11
    desired_y = 0.16
    desired_z = 0.1
    desired_yaw = 0
    
    # if yaw_constant_flag is Ture, then the desired yaw will set to be 0 for the whole flight
    yaw_constant_flag = False
    # whether take off
    take_off_flag = False
    # whether use position control from the mocap
    position_control_flag = True
    # axis activation threshold for the joystic
    axis_act_thre = 0.97

    # Crazyflie configuration
    # the URI of the Mark01 is 31/2M
    # the URI of the Mark02 is 101/2M
    # the URI of the small crazy flie is 23/2M
    URI = 'radio://0/31/2M'
    logging.basicConfig(level=logging.ERROR)

    # set the rigidbody ID
    rigid_body_ID = 1
    # flight_terminate_flag = False
    time_index = 0

    # set the inertia parameters
    inertia_dict = {
        'g': 9.81,
        'mp': 84.5/1000,   #gram, mass of the payload
        'mf': (216 + 10)/1000,  #gram, mass of the follower
        'alpha_1': math.pi/3,  #rad
        'alpha_2': math.pi/3,  #rad
    }

    g = inertia_dict['g']
    mp =  inertia_dict['mp']
    mf =  inertia_dict['mf']
    alpha_1 = inertia_dict['alpha_1']
    alpha_2 = inertia_dict['alpha_2']

    T2 = 1/(math.cos(alpha_2)/math.cos(alpha_1)*math.sin(alpha_1) + math.sin(alpha_2))*mp*g
    L = math.sqrt(T2**2 + (mf*g)**2 + 2*mf*g*T2*math.sin(alpha_2))
    gamma = math.asin(T2*math.cos(alpha_2)/L)

    # command ---> force mapping
    
    #roll_base = -gamma * 180.0 / math.pi
    #thrust_base = int((L + 14.937*g)*2125/g) + 2000
    thrust_base = 38000
    roll_base = -3.1
    # thrust_base = 41000
    # roll_base = 2
    
    thrust_count = 0
    roll_count = 0

    thrust_inc_flag = False
    thrust_dec_flag = False
    roll_inc_flag = False
    roll_dec_flag = False

    estimated_height = 0
    estimated_veloccity = 0

    # -----------------
    saver_1 = savemat.DataSaver(
        'roll_command',
        'pitch_command',
        # 'roll',
        # 'pitch',
        # 'P_x',
        # 'I_x',
        # 'D_x',
        # 'P_y',
        # 'I_y',
        # 'D_y',
        # 'e_x',
        # 'e_y',
        # 'T',
    )
    # -----------------

    # connect to the crazy flie and start the flight
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        """
        set the controller for this carzyflie in the firmware
        1 -> PID controller
        2 -> Mellinger controller
        3 -> INDI controller
        """
        scf.cf.param.set_value('stabilizer.controller', '1')
        time.sleep(0.1)
        scf.cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)
        start_time = time.time()
        abs_time = 0

        # loop
        with SyncLogger(scf, lg_stab) as logger:
            # while not flight_terminate_flag:
            for log_entry in logger:
                # print the data from the logging list
                timestamp = log_entry[0]
                # when acc_z is positive, meaning it points downwards
                log_acc_z = (log_entry[1].get('acc.z') - 1)*9.81
                log_thrust = log_entry[1].get('stabilizer.thrust')
                log_roll = log_entry[1].get('stabilizer.roll')

                logconf_name = log_entry[2]
                
                # print twice per second
                # if time_index % 100 == 0:
                #     # print('[%d][%s]: %s' % (timestamp, logconf_name, log_acc_z))
                #     print('thrust_count=', thrust_count, 'roll_count=', roll_count)

                time_index = time_index + 1
                abs_time_delayed = abs_time
                abs_time = time.time() - start_time
                # data = receiver.get_data_sync()
                time.sleep(sample_time)
                data = receiver.get_data()

                # get the data from the mocap
                dp.step(data[(rigid_body_ID-1)*14 : rigid_body_ID*14])
                joystick.step()

                # if trajectory_on:
                #     # my_trajectory.update()
                #     # desired_x, _, _, _, _ = my_trajectory.get_x()
                #     # desired_y, _, _, _, _ = my_trajectory.get_y()
                #     # desired_z, _, _, _, _ = my_trajectory.get_z()
                #     desired_yaw = Filter_pad_rx.filter(0)
                # else:
                #     # desired_x = desired_x + Filter_pad_ly.filter(joystick.get_axis_ly_ps4()*0.2)*0.01
                #     # desired_y = desired_y + Filter_pad_lx.filter(joystick.get_axis_lx_ps4()*0.2)*0.01
                #     # desired_z = desired_z + Filter_pad_ry.filter(- joystick.get_axis_ry_ps4()*0.2)*0.01
                #     desired_yaw = Filter_pad_rx.filter(0)

                # the position and yaw angle feedback from the MoCap
                yaw_feedback = math.atan2(2 * (dp.QW * dp.QZ + dp.QX * dp.QY), 1 - 2 * (dp.QZ * dp.QZ + dp.QY * dp.QY))
                X_feedback = Filter_x.filter(dp.X)
                Y_feedback = Filter_y.filter(dp.Y)
                Z_feedback = Filter_z.filter(dp.Z)

                # the desired position and yaw angle updated by the joystick, by integrating the commands
                desired_x = desired_x + joystick.get_axis_ly() * sample_time * 0.5
                desired_y = desired_y + joystick.get_axis_lx() * sample_time * 0.5
                desired_z = desired_z - joystick.get_axis_ry() * sample_time * 0.4
                if yaw_constant_flag:
                    desired_yaw = 0.0
                else:
                    desired_yaw = Filter_pad_rx.filter(desired_yaw + joystick.get_axis_rx() * sample_time * 20)
    

                # update the reference and get the controller output
                position_controller.update_reference(desired_x, desired_y, desired_z)

                u_x, u_y, u_z = position_controller.update_error(X_feedback, Y_feedback, Z_feedback)

                # mapping the roll, pithch, yaw and thrust
                pitch = -(u_x * math.cos(yaw_feedback) + u_y * math.sin(yaw_feedback)) / (math.cos(yaw_feedback) * math.cos(yaw_feedback) 
                          + math.sin(yaw_feedback) * math.sin(yaw_feedback))
                roll = (u_y * math.cos(yaw_feedback) - u_x * math.sin(yaw_feedback)) / (math.cos(yaw_feedback) * math.cos(yaw_feedback) 
                        + math.sin(yaw_feedback) * math.sin(yaw_feedback))
                thrust = limit_thrust(u_z)

                Yaw_error = limit_angular(limit_angular(desired_yaw) - limit_angular(yaw_feedback))
                yaw = - Yaw_error * 50
                # yaw = - Yaw_error * 75
                
                # to adjust the roll and thrust commands by the joystick when the position control is off
                thrust_inc_flag_prev = thrust_inc_flag
                thrust_dec_flag_prev = thrust_dec_flag
                roll_inc_flag_prev = roll_inc_flag
                roll_dec_flag_prev = roll_dec_flag
                
                # push the button_l to take off
                if joystick.get_button_l():
                    take_off_flag = True
                # push button Y only (with released LT) to activate the position control (get the position control back)
                if joystick.get_button_y() and joystick.get_axis_lt() < -axis_act_thre:
                    position_control_flag = True
                # push button Y and LT simutaneously to deactivate the position control until it's back
                if joystick.get_button_y() and joystick.get_axis_lt() > axis_act_thre:
                    position_control_flag = False

                # actually the rising edge detection
                # rb -> increase the thrust
                if joystick.get_button_rb():
                    thrust_inc_flag = True
                    if (not thrust_inc_flag_prev) and  thrust_inc_flag:
                        thrust_count = int(thrust_count + 1)
                else:
                    thrust_inc_flag = False 

                # lb -> decrease the thrust
                if joystick.get_button_lb():
                    thrust_dec_flag = True
                    if (not thrust_dec_flag_prev) and  thrust_dec_flag:
                        thrust_count = int(thrust_count - 1)
                else:
                    thrust_dec_flag = False    

                # b -> increase the roll
                if joystick.get_button_b():
                    roll_inc_flag = True
                    if (not roll_inc_flag_prev) and  roll_inc_flag:
                        roll_count = int(roll_count + 1)
                else:
                    roll_inc_flag = False 

                # a -> decrease the roll
                if joystick.get_button_a():
                    roll_dec_flag = True
                    if (not roll_dec_flag_prev) and  roll_dec_flag:
                        roll_count = int(roll_count - 1)
                else:
                    roll_dec_flag = False                
                           
                # send setpoint commands to the robot
                if not take_off_flag:
                    # not take off
                    scf.cf.commander.send_setpoint(0, 0, 0, 4000)
                else:
                    if not position_control_flag:
                        # the position control is off
                        roll_cmd = roll_base - roll_count * 0.1
                        thrust_cmd = int(thrust_base + thrust_count * 100)

                        scf.cf.commander.send_setpoint(roll_cmd, -0.3, 0, thrust_cmd)
                        if time_index % 50 == 0:
                            print('[%d][%s]: %s' % (timestamp, logconf_name, log_acc_z))
                            print('position_control_off', 'time = ', int(time_index/50), 
                                'roll_cmd=', roll_cmd, 'roll_actual = ', log_roll,
                               'thrust_cmd=', thrust_cmd, 'thrust_actual', log_thrust)   
                            # need to print the actual roll pitch yaw if necessary             
                    else:
                        # the position control is on

                        # acc_z = (-log_entry[1].get('acc.z') + 1)*9.81
                        # estimated_height = estimated_height + estimated_veloccity*sample_time + 0.5*acc_z*sample_time**2
                        # estimated_veloccity = acc_z*sample_time
                        scf.cf.commander.send_setpoint(roll, pitch, yaw, thrust)
                        # scf.cf.commander.send_setpoint(0, 0, 0, 30000)
                        # thrust_command = 30000
                        # scf.cf.param.set_value('motorPowerSet.enable', 1)
                        # scf.cf.param.set_value('motorPowerSet.m1', thrust_command)
                        # time.sleep(0.01)
                        # scf.cf.param.set_value('motorPowerSet.m2', thrust_command)
                        # time.sleep(0.01)
                        # scf.cf.param.set_value('motorPowerSet.m3', thrust_command)
                        # time.sleep(0.01)
                        # scf.cf.param.set_value('motorPowerSet.m4', thrust_command)
                        # time.sleep(0.01)

                        # print(log_entry[1].get('externalforce.T'))

                        # if time_index % 100 == 0:
                        #     # print('[%d][%s]: %s' % (timestamp, logconf_name, log_acc_z))
                        #     # print('position_control_on', 'time = ', int(time_index/100), 'thrust=', thrust, 
                        #     #     'acc_z = ', log_acc_z, 'roll_actual = ', log_roll) 
                        #     print('thrust_cmd = ', log_entry[1].get('stabilizer.thrust'),
                        #         ' ;acc_z = ', acc_z,
                        #         ' ;thrust_inNewton = ', log_entry[1].get('externalforce.T'),
                        #         ' ;estimated_height = ', estimated_height
                        #         )  

                # saver.add_elements(abs_time, dp.X, dp.Y, dp.Z, dp.QW, dp.QX, dp.QY, dp.QZ,
                                #    roll, pitch, yaw, thrust, yaw_feedback)
                saver_1.add_elements(roll,pitch)
                # if joystick.get_button_l1():
                # position_controller.integrator_enable()
                # trajectory_on = True

                # push X buttom to terminate flight
                if joystick.get_button_x():
                    scf.cf.commander.send_setpoint(0, 0, 0, 3000)
                    # scf.cf.param.set_value('motorPowerSet.enable', 0)

                    time.sleep(0.2)
                    scf.cf.commander.send_setpoint(0, 0, 0, 0)
                    break

    receiver.stop_thread()
    saver_1.save2mat('DATA/')
    # saver.save2mat('DataExchange/')
    joystick.quit()
