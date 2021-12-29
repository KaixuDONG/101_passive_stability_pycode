"""
this  is  the  program for  a single crazyflie using mocap,  with variables logger as well

checked in NOV 1, 2021, suitable for MacOS
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
from cooperative_transport.controllers import Creat_Agent

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# saturation funciton
def limit_thrust(thrust_cmd):
    if thrust_cmd > 65535:
        thrust_cmd = 65535
    if thrust_cmd < 0:
        thrust_cmd = 0
    return int(thrust_cmd)

# creat the crazyflie agent
leader = Creat_Agent()


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

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
    # lg_stab.add_variable('stabilizer.kfposz', 'float')
    # lg_stab.add_variable('stabilizer.kfvelz', 'float')
    lg_stab.add_variable('stateEstimate.roll', 'float')
    lg_stab.add_variable('stateEstimateZ.z', 'int16_t')
    lg_stab.add_variable('stateEstimateZ.vz', 'int16_t')
    

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
    leader.pos_x.des = 0.89
    leader.pos_y.des = -0.17
    leader.pos_z.des = 0.1
    leader.yaw.des = 0

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
    URI = 'radio://0/90/1M'
    logging.basicConfig(level=logging.ERROR)

    # set the rigidbody ID
    rigid_body_ID = 1
    
    # flight_terminate_flag = False
    time_index = 0

    # date saver, to be processed by Matlab
    saver_1 = savemat.DataSaver(
        'leader_pos_z_mocap',
        'leader_vel_z_mocap',
        'leader_pos_z_state',
        'leader_vel_z_state',
        # 'leader_pos_z_filter',
        # 'leader_vel_z_filter',
    )
    # -----------------

    leader_pos_z_prev = 0.0

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
        # unlock all the propellers
        scf.cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)
        start_time = time.time()
        abs_time = 0

        # loop
        with SyncLogger(scf, lg_stab) as logger:
            # while not flight_terminate_flag:
            for log_entry in logger:

                time_index = time_index + 1
                abs_time_delayed = abs_time
                abs_time = time.time() - start_time

                time.sleep(sample_time)
                data_mocap = receiver.get_data()

                # get the date from onboard logging
                data_log = log_entry[1]

                # get the data from the mocap
                dp.step(data_mocap[(rigid_body_ID-1)*14 : rigid_body_ID*14])
                joystick.step()

                # the position and yaw angle feedback from the MoCap
                leader.yaw.fdk = math.atan2(2 * (dp.QW * dp.QZ + dp.QX * dp.QY), 1 - 2 * (dp.QZ * dp.QZ + dp.QY * dp.QY))
                leader.pos_x.fdk = Filter_x.filter(dp.X)
                leader.pos_y.fdk = Filter_y.filter(dp.Y)
                leader.pos_z.fdk = Filter_z.filter(dp.Z)

                # the desired position and yaw angle updated by the joystick, by integrating the commands
                leader.pos_x.des +=  joystick.get_axis_ly() * sample_time * 0.5
                leader.pos_y.des +=  joystick.get_axis_lx() * sample_time * 0.5
                leader.pos_z.des -= joystick.get_axis_ry() * sample_time * 0.4
                
                if yaw_constant_flag:
                   leader.yaw.des = 0.0
                else:
                    leader.yaw.des = Filter_pad_rx.filter(leader.yaw.des + joystick.get_axis_rx() * sample_time * 20)
    

                # update the reference and get the controller output
                position_controller.update_reference( leader.pos_x.des ,  leader.pos_y.des ,  leader.pos_z.des )

                u_x, u_y, u_z = position_controller.update_error(leader.pos_x.fdk, leader.pos_y.fdk, leader.pos_z.fdk)

                # mapping the roll, pithch, yaw and thrust
                leader.pitch.des = -(u_x * math.cos(leader.yaw.fdk) + u_y * math.sin(leader.yaw.fdk)) / (math.cos(leader.yaw.fdk) * math.cos(leader.yaw.fdk) 
                          + math.sin(leader.yaw.fdk) * math.sin(leader.yaw.fdk))
                leader.roll.des = (u_y * math.cos(leader.yaw.fdk) - u_x * math.sin(leader.yaw.fdk)) / (math.cos(leader.yaw.fdk) * math.cos(leader.yaw.fdk) 
                        + math.sin(leader.yaw.fdk) * math.sin(leader.yaw.fdk))
                leader.thrus_cmd.des = limit_thrust(u_z)

                Yaw_error = limit_angular(limit_angular(leader.yaw.des) - limit_angular(leader.yaw.fdk))
                yaw = - Yaw_error * 50
                
                # push the button_l to take off
                if joystick.get_button_l():
                    take_off_flag = True

                # flight process
                if not take_off_flag:
                    # not take off
                    scf.cf.commander.send_setpoint(0, 0, 0, 4000)
                else:
                    scf.cf.commander.send_setpoint(leader.roll.des, leader.pitch.des, yaw, leader.thrus_cmd.des)
                    print(leader.thrus_cmd.des)
                # only save the data when 
                saver_1.add_elements(leader.pos_z.fdk, 
                                     (leader.pos_z.fdk - leader_pos_z_prev)/sample_time,
                                     (data_log['stateEstimateZ.z'])/1000,
                                     (data_log['stateEstimateZ.vz'])/1000,
                                    #  data_log['stabilizer.kfposz'],
                                    #  data_log['stabilizer.kfvelz'],
                                    )

                leader_pos_z_prev = leader.pos_z.fdk

                # push X buttom to terminate flight
                if joystick.get_button_x():
                    scf.cf.commander.send_setpoint(0, 0, 0, 3000)

                    time.sleep(0.2)
                    scf.cf.commander.send_setpoint(0, 0, 0, 0)
                    break

    receiver.stop_thread()
    saver_1.save2mat('DATA_NEW_LEADER/')
    joystick.quit()
