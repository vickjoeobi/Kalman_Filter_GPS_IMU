#!/usr/bin/env python
# -*- coding: utf-8 -*-

HOST = "localhost"
PORT = 4223
UID = "XXYYZZ" # Change XXYYZZ to the UID of your IMU Brick 2.0
GPS_UID = "XYZ" # Change XYZ to the UID of your GPS Bricklet 3.0

from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_imu_v2 import BrickIMUV2
from tinkerforge.bricklet_gps_v3 import BrickletGPSV3
import numpy as np
import math


# Callback function for all data callback
def cb_all_data(acceleration, angular_velocity, euler_angle, quaternion, calibration_status):
    print("Acceleration [X]: " + str(acceleration[0]/100.0) + " m/s²")
    print("Acceleration [Y]: " + str(acceleration[1]/100.0) + " m/s²")
    print("Acceleration [Z]: " + str(acceleration[2]/100.0) + " m/s²")
    print("Angular Velocity [X]: " + str(angular_velocity[0]/16.0) + " °/s")
    print("Angular Velocity [Y]: " + str(angular_velocity[1]/16.0) + " °/s")
    print("Angular Velocity [Z]: " + str(angular_velocity[2]/16.0) + " °/s")
    print("Euler Angle [Heading]: " + str(euler_angle[0]/16.0) + " °")
    print("Euler Angle [Roll]: " + str(euler_angle[1]/16.0) + " °")
    print("Euler Angle [Pitch]: " + str(euler_angle[2]/16.0) + " °")
    print("Quaternion [W]: " + str(quaternion[0]/16383.0))
    print("Quaternion [X]: " + str(quaternion[1]/16383.0))
    print("Quaternion [Y]: " + str(quaternion[2]/16383.0))
    print("Quaternion [Z]: " + str(quaternion[3]/16383.0))
    print("Calibration Status: " + format(calibration_status, "08b"))
    print("")

    #append the data to csv file
    data = np.array([acceleration[0]/100.0,acceleration[1]/100.0,acceleration[2]/100.0,angular_velocity[0]/16.0,angular_velocity[1]/16.0,angular_velocity[2]/16.0,euler_angle[0]/16.0,euler_angle[1]/16.0,euler_angle[2]/16.0,quaternion[0]/16383.0,quaternion[1]/16383.0,quaternion[2]/16383.0,quaternion[3]/16383.0])
    
    #add data to existing csv file
    with open('imu_data.csv', 'a') as f:
        np.savetxt(f, data, delimiter=",")


def cb_coordinates(latitude, ns, longitude, ew):
    print("Latitude: " + str(latitude/1000000.0) + " °")
    print("N/S: " + ns)
    print("Longitude: " + str(longitude/1000000.0) + " °")
    print("E/W: " + ew)
    print("")

    #Convert the longitude and latitude to x,y,z coordinates
    lat = latitude/1000000.0
    lon = longitude/1000000.0
    r = 6371.0

    x = r * math.cos(lat) * math.cos(lon)
    y = r * math.cos(lat) * math.sin(lon)
    z = r * math.sin(lat)

    print("X: " + str(x) + " m")
    print("Y: " + str(y) + " m")
    print("Z: " + str(z) + " m")
    print("")

    #append the data to csv file
    data = np.array([x,y,z])
    
    #add data to existing csv file
    with open('gps_data.csv', 'a') as f:
        np.savetxt(f, data, delimiter=",")


if __name__ == "__main__":
    ipcon = IPConnection() # Create IP connection
    imu = BrickIMUV2(UID, ipcon) # Create device object
    gps = BrickletGPSV3(GPS_UID, ipcon) # Create device object

    ipcon.connect(HOST, PORT) # Connect to brickd
    # Don't use device before ipcon is connected

    # Register all data callback to function cb_all_data
    imu.register_callback(imu.CALLBACK_ALL_DATA, cb_all_data)

    # Register coordinates callback to function cb_coordinates
    gps.register_callback(gps.CALLBACK_COORDINATES, cb_coordinates)

    # Set period for all data callback to 0.1s (100ms)
    imu.set_all_data_period(100)

    # Set period for coordinates callback to 1s (1000ms)
    # Note: The coordinates callback is only called every second
    #       if the coordinates has changed since the last call!
    gps.set_coordinates_callback_period(1000)

    input("Press key to exit\n") # Use raw_input() in Python 2
    ipcon.disconnect()