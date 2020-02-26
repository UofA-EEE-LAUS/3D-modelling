import time
import sys
from NatNetClient import NatNetClient
import logging
import time
from threading import Thread
import numpy as np
import cflib
from cflib.crazyflie import Crazyflie

logging.basicConfig(level=logging.ERROR)

# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, latency, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
    # print( "Received frame", frameNumber )
	pass

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation ):
    # print( "Received frame for rigid body", id )
	if id==1:
		global pos_x, pos_y, pos_z, att_x, att_y, att_z
		pos_x = position[0]
		pos_y = position[1]
		pos_z = position[2]
		att_x = rotation[0]
		att_y = rotation[1]
		att_z = rotation[2]
	else:
		pass

pos_x, pos_y, pos_z = 0.0, 0.0, 0.0
att_x, att_y, att_z = 0.0, 0.0, 0.0
# This will create a new NatNet client
streamingClient = NatNetClient()
# Configure the streaming client to call our rigid body handler on the emulator to send data out.
streamingClient.newFrameListener = receiveNewFrame
streamingClient.rigidBodyListener = receiveRigidBodyFrame
# Start up the streaming client now that the callbacks are set up.
# This will run perpetually, and operate on a separate thread.
streamingClient.run()

print('Start tracking')
start_time = time.time()

# Load the trajectory data (in 500Hz)
positions_x = np.loadtxt('positions_x.txt')
positions_y = np.loadtxt('positions_y.txt')


# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(theta):
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])

    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])

    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])

    R = np.dot(R_z, np.dot(R_y, R_x))

    return R


class PositionControlMocap:
    """Class that connects to a Crazyflie, lets it take off and sends set-points based on OptiTrack data."""

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)

        print('Connecting to %s' % link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._follow_trajectory).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)

    def _follow_trajectory(self):
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        for i in range(len(positions_x)):

            # Current position setpoint
            x_ref = positions_x[i]
            y_ref = positions_y[i]
            z_ref = 2
            # Current mocap position and orientation
            yaw = att_z
            pitch = att_y
            roll = att_x
            x_hat = pos_x
            y_hat = pos_y
            z_hat = pos_z


            self._cf.commander.send_setpoint()
            time.sleep(0.002)

        self._cf.commander.send_setpoint(0, 0, 0, 0)
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.05)
        self._cf.close_link()


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) > 0:
        le = PositionControlMocap(available[0][0])
    else:
        print('No Crazyflies found, cannot run example')

