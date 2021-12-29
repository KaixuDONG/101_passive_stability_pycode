"""
this is the code in the first version
which has most limited functions, like using mocap to control a robot 

but without the functionality of logging data from the robot

-> Nov 1, 2021
----------!!!!------
this code is a very old version 

and it should not be used again
"""


import logging
import time
import math

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from rislab_lib.mocap_udp import UdpReceiver
from rislab_lib.Joystick import joystick
from rislab_lib.savemat import savemat
from rislab_lib.controllers import pid_3d
from rislab_lib.data_processor import GeneralFcn
from rislab_lib.controllers import trajectory_loader

from cooperative_transport.controllers import PID_ControllerThreeAixs
from cooperative_transport.wheel_functions import limit_angular


def limit_thrust(thrust_cmd):
    if thrust_cmd > 65535:
        thrust_cmd = 65535
    if thrust_cmd < 0:
        thrust_cmd = 0
    return int(thrust_cmd)


if __name__ == '__main__':
    saver = savemat.DataSaver('Abs_time', 'X', 'Y', 'Z', 'QW', 'QX', 'QY', 'QZ', 'roll', 'pitch', 'yaw', 'thrust','angleY')

    # my_trajectory = trajectory_loader.Trajectory('/CFrobot/DataExchange/Trajecotries/T1.mat')
    # my_trajectory.update()
    # udp receiver
    receiver = UdpReceiver.UdpRigidBodies()
    sample_rate = receiver.get_sample_rate()
    # sample_rate = 50
    sample_time = 1 / sample_rate
    receiver.start_thread()

    # data processor
    dp = GeneralFcn.RealTimeProcessor()
    Filter_x = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)
    Filter_y = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)
    Filter_z = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)
    
    Filter_pad_ly = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=65, fs=sample_rate)
    Filter_pad_lx = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=65, fs=sample_rate)
    Filter_pad_ry = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=65, fs=sample_rate)
    Filter_pad_rx = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=65, fs=sample_rate)

    # controller parameter
    trajectory_on = False
    position_controller = PID_ControllerThreeAixs(sample_time,
                                       10, 1, 20, 0,
                                       10, 1, 20, 0,
                                       10000, 500, 10000, 40000,)
    desired_x = -0.32
    desired_y = 0.06
    desired_z = 1.35
    desired_yaw = 0

    # Crazyflie configuration
    # the URI of the Mark01 is 80/2M
    # the URI of the Mark02 is 101/2M
    URI = 'radio://0/80/2M'
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    flight_terminate_flag = False
    time_index = 0

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.param.set_value('stabilizer.controller','1')
        time.sleep(0.1)
        scf.cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)
        start_time = time.time()
        abs_time = 0
        
        # loop
        while not flight_terminate_flag:
            time_index = time_index + 1
            abs_time_delayed = abs_time
            abs_time = time.time() - start_time
            data = receiver.get_data_sync()
            # time.sleep(1/50)
            # data = receiver.get_data()
            dp.step(data)
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

            # the desired position and yaw angle updated by the joystick
            desired_x = desired_x + joystick.get_axis_ly()*sample_time*0.4
            desired_y = desired_y + joystick.get_axis_lx()*sample_time*0.4
            desired_z = desired_z - joystick.get_axis_ry()*sample_time*0.3
            desired_yaw = Filter_pad_rx.filter(desired_yaw + joystick.get_axis_rx()*sample_time*10)

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
            
            # send setpoint commands to the robot
            # push Y, lb, rb to send constant command to the robot with disabled position control
            if joystick.get_button_y():
                scf.cf.commander.send_setpoint(4, 0, 0, 39000)    

            elif joystick.get_button_lb():
                scf.cf.commander.send_setpoint(3, 0, 0, 38000)    

            elif joystick.get_button_rb():
                scf.cf.commander.send_setpoint(3.5, 0, 0, 37000)   
            
            else:
                # the position control is on
                scf.cf.commander.send_setpoint(roll, pitch, yaw, thrust)

            saver.add_elements(abs_time, dp.X, dp.Y, dp.Z, dp.QW, dp.QX, dp.QY, dp.QZ,
                               roll, pitch, yaw, thrust, yaw_feedback)
            
            
            # if joystick.get_button_l1():
                # position_controller.integrator_enable()
                # trajectory_on = True
                        
            # push X buttom to terminate flight
            if joystick.get_button_x():
                flight_terminate_flag = True
                scf.cf.commander.send_setpoint(0, 0, 0, 30000)
                time.sleep(0.2)
                scf.cf.commander.send_setpoint(0, 0, 0, 0)

            if time_index % 100 == 0:
                print(desired_yaw, Yaw_error)


    receiver.stop_thread()
    # saver.save2mat('DataExchange/')
    joystick.quit()
