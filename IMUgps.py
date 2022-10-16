#!/usr/bin/env python
# -*- coding: utf-8 -*-

HOST = "localhost"
PORT = 4223
UID = "6ypCku" # Change XXYYZZ to the UID of your IMU Brick 2.0
gpsUID = "21NG"

from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_imu_v2 import BrickIMUV2
from tinkerforge.bricklet_gps_v3 import BrickletGPSV3

# Callback function for all data callback
def cb_all_data(acceleration, magnetic_field, angular_velocity, euler_angle, quaternion,
                linear_acceleration, gravity_vector, temperature, calibration_status):
    print("Acceleration [X]: " + str(acceleration[0]/100.0) + " m/s²")
    print("Acceleration [Y]: " + str(acceleration[1]/100.0) + " m/s²")
    print("Acceleration [Z]: " + str(acceleration[2]/100.0) + " m/s²")
    print("Magnetic Field [X]: " + str(magnetic_field[0]/16.0) + " µT")
    print("Magnetic Field [Y]: " + str(magnetic_field[1]/16.0) + " µT")
    print("Magnetic Field [Z]: " + str(magnetic_field[2]/16.0) + " µT")
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
    print("Linear Acceleration [X]: " + str(linear_acceleration[0]/100.0) + " m/s²")
    print("Linear Acceleration [Y]: " + str(linear_acceleration[1]/100.0) + " m/s²")
    print("Linear Acceleration [Z]: " + str(linear_acceleration[2]/100.0) + " m/s²")
    print("Gravity Vector [X]: " + str(gravity_vector[0]/100.0) + " m/s²")
    print("Gravity Vector [Y]: " + str(gravity_vector[1]/100.0) + " m/s²")
    print("Gravity Vector [Z]: " + str(gravity_vector[2]/100.0) + " m/s²")
    print("Temperature: " + str(temperature) + " °C")
    print("Calibration Status: " + format(calibration_status, "08b"))
    print("")
    
def cb_coordinates(latitude, ns, longitude, ew):
    print("Latitude: " + str(latitude/1000000.0) + " °")
    print("N/S: " + ns)
    print("Longitude: " + str(longitude/1000000.0) + " °")
    print("E/W: " + ew)
    print("")

if __name__ == "__main__":
    ipcon = IPConnection() # Create IP connection
    imu = BrickIMUV2(UID, ipcon) # Create device object
    gps = BrickletGPSV3(gpsUID, ipcon)

    ipcon.connect(HOST, PORT) # Connect to brickd
    # Don't use device before ipcon is connected

    # Register all data callback to function cb_all_data
    gps.register_callback(gps.CALLBACK_COORDINATES, cb_coordinates)
    imu.register_callback(imu.CALLBACK_ALL_DATA, cb_all_data)


    # Set period for all data callback to 0.1s (100ms)
    imu.set_all_data_period(100)
    gps.set_coordinates_callback_period(1000)

    input("Press key to exit\n") # Use raw_input() in Python 2
    ipcon.disconnect()

