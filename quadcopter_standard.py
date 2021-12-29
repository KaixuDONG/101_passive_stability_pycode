"""
this is the most prototype code for the crazy flie python code
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

    # controller
    trajectory_on = False
    position_controller = pid_3d.Pid3D(sample_time,
                                       10, 1, 20, 0,
                                       10, 1, 20, 0,
                                       10000, 3000, 10000, 40000,)
    desired_x = -0.5
    desired_y = 0
    desired_z = 0.15

    # Crazyflie configuration
    URI = 'radio://0/80/2M'
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    done = False
    flag = 0

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)
        start_time = time.time()
        abs_time = 0

        while not done:
            flag = flag + 1
            abs_time_delayed = abs_time
            abs_time = time.time() - start_time
            data = receiver.get_data_sync()
            dp.step(data)
            angle_yaw = math.atan2(2 * (dp.QW * dp.QZ + dp.QX * dp.QY), 1 - 2 * (dp.QZ * dp.QZ + dp.QY * dp.QY))
            joystick.step()

            if trajectory_on:
                # my_trajectory.update()
                # desired_x, _, _, _, _ = my_trajectory.get_x()
                # desired_y, _, _, _, _ = my_trajectory.get_y()
                # desired_z, _, _, _, _ = my_trajectory.get_z()
                desired_yaw = Filter_pad_rx.filter(0)
            else:
                # desired_x = desired_x + Filter_pad_ly.filter(joystick.get_axis_ly_ps4()*0.2)*0.01
                # desired_y = desired_y + Filter_pad_lx.filter(joystick.get_axis_lx_ps4()*0.2)*0.01
                # desired_z = desired_z + Filter_pad_ry.filter(- joystick.get_axis_ry_ps4()*0.2)*0.01
                desired_yaw = Filter_pad_rx.filter(0)



            position_controller.update_reference(desired_x, desired_y, desired_z)

            X_f = Filter_x.filter(dp.X)
            Y_f = Filter_y.filter(dp.Y)
            Z_f = Filter_z.filter(dp.Z)

            u_x, u_y, u_z = position_controller.update_error(X_f, Y_f, Z_f)

            pitch = -(u_x * math.cos(angle_yaw) + u_y * math.sin(angle_yaw)) / (math.cos(angle_yaw) * math.cos(angle_yaw)
                                                                                + math.sin(angle_yaw) * math.sin(angle_yaw))
            roll = (u_y * math.cos(angle_yaw) - u_x * math.sin(angle_yaw)) / (math.cos(angle_yaw) * math.cos(angle_yaw)
                                                                              + math.sin(angle_yaw) * math.sin(angle_yaw))
            thrust = limit_thrust(u_z)

            Yaw_error = desired_yaw - angle_yaw
            if Yaw_error > math.pi:
                Yaw_error = Yaw_error - 2 * math.pi
            elif Yaw_error < -math.pi:
                Yaw_error = Yaw_error + 2 * math.pi
            yaw = - Yaw_error * 50

            scf.cf.commander.send_setpoint(roll, pitch, yaw, thrust)

            saver.add_elements(abs_time, dp.X, dp.Y, dp.Z, dp.QW, dp.QX, dp.QY, dp.QZ,
                               roll, pitch, yaw, thrust, angle_yaw)
            if joystick.get_button_l1():
                position_controller.integrator_enable()
                trajectory_on = True
            if joystick.get_button_x():
                done = True
                scf.cf.commander.send_setpoint(0, 0, 5, 32500)
                time.sleep(5)
                scf.cf.commander.send_setpoint(0, 0, 0, 0)

            if flag % 100 == 0:
                print(desired_yaw, Yaw_error )


    receiver.stop_thread()
    saver.save2mat('DataExchange/')
    joystick.quit()
