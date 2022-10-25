#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""This function extracts the IMU and GPS data from the Tinkerforge Bricklets"""


HOST = "localhost"
PORT = 4223
READ_PERIOD = 100 # in ms
READ_PERIODgps = 1000



from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_imu_v2 import BrickIMUV2
from tinkerforge.bricklet_gps_v3 import BrickletGPSV3

import time
import sys
import csv
from datetime import date

imu_uid = "ypCku"
gps_uid = "21NG"

def cb_enumerate(uid, connected_uid, position, hardware_version, firmware_version,
                 device_identifier, enumeration_type):
    if enumeration_type == IPConnection.ENUMERATION_TYPE_DISCONNECTED:
        return

    # Set imu_uid if any IMU is discovered, we assume that there is only 
    if device_identifier == 18:
        global imu_uid
        imu_uid = uid
        
def kalman_filter_fussion_imu_gps(x, P, measurement, R, motion, Q, B):
    """
    Parameters:
    x: initial state 4-tuple of location and velocity: (x0, x1, x2, x3)
    P: initial uncertainty convariance matrix
    measurement: observed position
    R: measurement noise 
    motion: external motion added to state vector x
    Q: motion noise (same shape as P)
    B: next state function: x_prime = B*x
    """
    # UPDATE x, P based on measurement m
    # distance between measured and current position-belief
    y = np.matrix(measurement).T - B * x
    S = B * P * B.T + R # residual convariance
    K = P * B.T * S.I    # Kalman gain
    x = x + K*y
    I = np.matrix(np.eye(P.shape[0])) # identity matrix
    P = (I - K*B)*P

    # PREDICT x, P based on motion
    x = B*x + motion
    P = B*P*B.T + Q

    return x, P 

if __name__ == "__main__":
    ipcon = IPConnection() # Create IP connection
    ipcon.connect(HOST, PORT) # Connect to brickd

    # Register Enumerate Callback
    ipcon.register_callback(IPConnection.CALLBACK_ENUMERATE, cb_enumerate)

    # Trigger Enumerate
    ipcon.enumerate()

    # Wait for 1 second enumeration
    time.sleep(1)


    imu = BrickIMUV2(imu_uid, ipcon) # Create device object
    gps = BrickletGPSV3(gps_uid, ipcon) # Create device object


    try:
        while True:
            time.sleep(1.0/(1000.0/READ_PERIOD))

            acceleration, magnetic_field, angular_velocity, \
            euler_angle, quaternion, linear_acceleration, gravity_vector, \
            temperature, calibration_status = imu.get_all_data()

            latitude, ns, longitude, ew = gps.get_coordinates()

            print("Acceleration [X]: " + str(acceleration[0]/100.0) + " g")
            print("Acceleration [Y]: " + str(acceleration[1]/100.0) + " g")
            print("Acceleration [Z]: " + str(acceleration[2]/100.0) + " g")
            print("")
            print("Longitude: " + str(longitude/1000000.0) + " °")
            print("Latitude: " + str(latitude/1000000.0) + " °")
            print("")

    except KeyboardInterrupt:
        pass
        
    ipcon.disconnect()
