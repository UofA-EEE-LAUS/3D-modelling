import math
import os
import time
from NatNetClient import NatNetClient
import logging
import time
from threading import Thread
import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from twisted.internet import reactor, threads
from threading import Timer
import xmltodict
import numpy as np
from helpers import convert_coords_to_setpoint, crazyflie_reset_estimator, print_status

# Settings
CRAZYFLIE_URI = "radio://0/100/250K"
CRAZYFLIE_RIGIDBODY_ID = "1"
FRAME_LOSS_THRESHOLD = 3
HOME_POSITION = (0.0, 0.0, 1, 0.0)  # (X, Y, Z, Yaw)
AWAY_POSITION = (0.1, 0.1, 1, 0.0)  # (X, Y, Z, Yaw)

# Initializing
scf = None
trackingFramesLost = 0
target = HOME_POSITION
flyAway = False


# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame(frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, latency, timecode, timecodeSub, timestamp, isRecording,
                    trackedModelsChanged):
    # print( "Received frame", frameNumber )
    pass

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame(id, position, rotation):
    # print( "Received frame for rigid body", id )
    if id == 1:
        global pos_x, pos_y, pos_z, att_x, att_y, att_z
        global scf, CRAZYFLIE_RIGIDBODY_ID, trackingFramesLost
        prev_pos_x = pos_x
        prev_pos_y = pos_y
        prev_pos_z = pos_z
        pos_x = position[0]
        pos_y = -position[2]
        pos_z = position[1]
        frame_rate = 202
        vel_x = (pos_x - prev_pos_x) * frame_rate
        vel_y = (pos_y - prev_pos_y) * frame_rate
        vel_z = (pos_z - prev_pos_z) * frame_rate
        att_x = rotation[0]
        att_y = rotation[1]
        att_z = rotation[2]

        # Increment frame loss counter if anything is wrong
        if not scf:
            trackingFramesLost += 1
            return

        # If OptiTrack loses tracking it may return `NaN` which can crash Crazyflie if sent
        # In case of `Nan` values, don't send to Crazyflie, and increment frame loss counter

        # print('processing position data')
        if math.isnan(pos_x):
            trackingFramesLost += 1
        else:
            scf.cf.extpos.send_extpos(pos_x, pos_y, pos_z)
            scf.cf.extpos.send_extvel(vel_x, vel_y, vel_z)
            trackingFramesLost = 0
    else:
        pass

def stab_log_data(timestamp, data, logconf):
    """Callback from the log API when data arrives"""

    print('[%d][%s]: %s' % (timestamp, logconf.name, data))

def stab_log_error(logconf, msg):
    """Callback from the log API when an error occurs"""
    print('Error when logging %s: %s' % (logconf.name, msg))

def crazyflie_controller():
    """Crazyflie flight controller - initializes position estimator and calls for flight instructions"""

    global scf, CRAZYFLIE_URI, flyAway

    # Reset target before liftoff for safety
    flyAway = False
    try:
        with SyncCrazyflie(CRAZYFLIE_URI) as _scf:
            # Update global Synchronous Crazyflie object when connected
            scf = _scf
            if not scf:
                print("Crazyflie is not True! Terminating.")
                os._exit(1)
            else:
                print("Connected to Crazyflie at", CRAZYFLIE_URI, "- resetting position estimator...")
                # crazyflie_reset_estimator(scf)
                time.sleep(0.5)
                # The definition of the logconfig can be made before connecting

                # lg_stab = LogConfig(name='Stabilizer', period_in_ms=50)
                # lg_stab.add_variable('stabilizer.roll', 'float')
                # lg_stab.add_variable('stabilizer.pitch', 'float')
                # lg_stab.add_variable('stabilizer.yaw', 'float')


                # lg_gyroacc = LogConfig(name='GyroAcc', period_in_ms=50)
                # lg_gyroacc.add_variable('gyro.x', 'float')
                # lg_gyroacc.add_variable('gyro.y', 'float')
                # lg_gyroacc.add_variable('gyro.z', 'float')
                # lg_gyroacc.add_variable('acc.x', 'float')
                # lg_gyroacc.add_variable('acc.y', 'float')
                # lg_gyroacc.add_variable('acc.z', 'float')

                lg_lopo = LogConfig(name='Lopo', period_in_ms=100)
                lg_lopo.add_variable('stateEstimate.x', 'float')
                lg_lopo.add_variable('stateEstimate.y', 'float')
                lg_lopo.add_variable('stateEstimate.z', 'float')
                lg_lopo.add_variable('stateEstimate.vx', 'float')
                lg_lopo.add_variable('stateEstimate.vy', 'float')
                lg_lopo.add_variable('stateEstimate.vz', 'float')
                # lg_lopo.add_variable('ranging.distance0', 'float')
                # lg_lopo.add_variable('ranging.distance1', 'float')
                # lg_lopo.add_variable('ranging.distance2', 'float')
                # lg_lopo.add_variable('ranging.distance3', 'float')

                try:
                    # scf.cf.log.add_config(lg_stab)
                    # # This callback will receive the data
                    # lg_stab.data_received_cb.add_callback(stab_log_data)
                    # # This callback will be called on errors
                    # lg_stab.error_cb.add_callback(stab_log_error)
                    # # Start the logging
                    # lg_stab.start()

                    # scf.cf.log.add_config(lg_gyroacc)
                    # # This callback will receive the data
                    # lg_gyroacc.data_received_cb.add_callback(stab_log_data)
                    # # This callback will be called on errors
                    # lg_gyroacc.error_cb.add_callback(stab_log_error)
                    # # Start the logging
                    # lg_gyroacc.start()

                    scf.cf.log.add_config(lg_lopo)
                    # This callback will receive the data
                    lg_lopo.data_received_cb.add_callback(stab_log_data)
                    # This callback will be called on errors
                    lg_lopo.error_cb.add_callback(stab_log_error)
                    # Start the logging
                    lg_lopo.start()
                    print('logging started')
                except KeyError as e:
                    print('Could not start log configuration,'
                          '{} not found in TOC'.format(str(e)))
                except AttributeError as e:
                    print('Could not add Stabilizer log config, bad configuration.')
                except Exception as e:
                    print(e)

                # Start a timer to disconnect in 10s
                # time.sleep(10)
                # scf.cf.extpos.send_extvel(4, 4, 4)
                # time.sleep(2)
                # os._exit(1)
                crazyflie_fly()
    except Exception as e:
        print("Terminating due to error while initializing flight controller:", str(e))
        os._exit(1)


def crazyflie_fly():
    """Provides flight instructions to Crazyflie based on global `fly` variable"""

    global scf, FRAME_LOSS_THRESHOLD, trackingFramesLost

    cf = scf.cf
    cf.param.set_value('flightmode.posSet', '1')

    # Crazyflie needs to be sent a setpoint at least twice a second or it will stop
    for i in range(0, len(positions_x), 100):
        # Check if tracking is good
        if trackingFramesLost <= FRAME_LOSS_THRESHOLD:
            # Current position setpoint
            x_ref = positions_x[i]
            y_ref = positions_y[i]
            z_ref = 2
            target = (x_ref, y_ref, z_ref, 0)
            # print_status("Setting position {}".format(target))
            # setpoint = convert_coords_to_setpoint(target)
            cf.commander.send_position_setpoint(*target)
            time.sleep(0.2)
        else:
            cf.commander.send_stop_setpoint()
            print("Tracking lost, terminating.")
            os._exit(1)

    target = (0, 0, 0.5, 0)
    # setpoint = convert_coords_to_setpoint(target)
    cf.commander.send_position_setpoint(*target)
    time.sleep(3)
    target = (0, 0, 0, 0)
    # setpoint = convert_coords_to_setpoint(target)
    cf.commander.send_position_setpoint(*target)
    time.sleep(1)
    cf.commander.send_stop_setpoint()
    print('done')
    os._exit(1)

if __name__ == '__main__':
    print("~ OptiTrack x CrazyFlie ~")

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    logging.basicConfig(level=logging.ERROR)
    pos_x, pos_y, pos_z = 0.0, 0.0, 0.0
    att_x, att_y, att_z = 0.0, 0.0, 0.0

    # Load the trajectory data (in 500Hz)
    positions_x = np.loadtxt('positions_x_1.txt')
    positions_y = np.loadtxt('positions_y_1.txt')

    # This will create a new NatNet client
    streamingClient = NatNetClient()
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streamingClient.newFrameListener = receiveNewFrame
    streamingClient.rigidBodyListener = receiveRigidBodyFrame

    print('starting crazyflie controller thread')
    # Start Crazyflie flight controller on a new thread
    d = threads.deferToThread(crazyflie_controller)
    streamingClient.run()
    reactor.run()



