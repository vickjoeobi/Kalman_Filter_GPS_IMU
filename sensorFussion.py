#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import sys
import csv
from datetime import date
from turtle import speed



HOST = "localhost"
PORT = 4223
IMU_UID = "6ypCku" # Change XXYYZZ to the UID of your IMU Brick 2.0
GPS_UID = "21NG" # Change XYZ to the UID of your GPS Bricklet 3.0
READ_PERIOD = 100 # in ms

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_gps_v3 import BrickletGPSV3
from tinkerforge.brick_imu_v2 import BrickIMUV2

IMU_DATA_TITLE = [
    'Time (unixtime)', 
    'Acc_X', 'Acc_Y', 'Acc_Z', 
    'Ang. Vel. X (o. pitch)', 'Ang. Vel. Y (o. roll)', 'Ang. Vel. Z (o. head)', 
    'Euler A. X', 'Euler A. Y', 'Euler A. Z', 
    'Lin. Acc. X', 'Lin. Acc. Y', 'Lin. Acc. Z', 
    'Grav. Vect. X', 'Grav. Vect. Y', 'Grav. Vect. Z', 
    'Temp'
]

def cb_enumerate(uid, connected_uid, position, hardware_version, firmware_version,
                 device_identifier, enumeration_type):
    if enumeration_type == IPConnection.ENUMERATION_TYPE_DISCONNECTED:
        return

    # Set imu_uid if any IMU is discovered, we assume that there is only 
    if device_identifier == 18:
        global imu_uid
        imu_uid = uid

if __name__ == "__main__":
    ipcon = IPConnection() # Create IP connection
    ipcon.connect(HOST, PORT) # Connect to brickd

    # Register Enumerate Callback
    ipcon.register_callback(IPConnection.CALLBACK_ENUMERATE, cb_enumerate)

    # Trigger Enumerate
    ipcon.enumerate()

    # Wait for 1 second enumeration
    time.sleep(1)

    gps = BrickletGPSV3(GPS_UID, ipcon) # Create device object
    imu = BrickIMUV2(IMU_UID, ipcon) # Create device object

    csv_file = open('log_imu_{0}_{1}.csv'.format(imu_uid, date.today()), 'w')
    csv_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL)
    csv_writer.writerow(IMU_DATA_TITLE)

    csv_file_gps = open('log_gps_{0}_{1}.csv'.format(GPS_UID, date.today()), 'w')
    csv_writer_gps = csv.writer(csv_file_gps, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL)
    csv_writer_gps.writerow(['Time (unixtime)', 'Latitude', 'NS', 'Longitude', 'EW', 'Altitude', 'PDOP', 'HDOP', 'VDOP', 'EPE'])



    print("Logger started. Use ctr+c to exit.")

    while True:
        # Read imu data
        imu_data = imu.get_all_data()
        # Read gps data
        gps_data = gps.get_coordinates()

        # Write imu data
        csv_writer.writerow([
            time.time(), 
            imu_data.acceleration[0], imu_data.acceleration[1], imu_data.acceleration[2], 
            imu_data.angular_velocity[0], imu_data.angular_velocity[1], imu_data.angular_velocity[2], 
            imu_data.euler_angle[0], imu_data.euler_angle[1], imu_data.euler_angle[2], 
            imu_data.linear_acceleration[0], imu_data.linear_acceleration[1], imu_data.linear_acceleration[2], 
            imu_data.gravity_vector[0], imu_data.gravity_vector[1], imu_data.gravity_vector[2], 
            imu_data.temperature
        ])




        # Write gps data
        csv_writer_gps.writerow([
            time.time(), 
            gps_data.latitude, gps_data.ns, gps_data.longitude, gps_data.ew, gps_data.altitude, gps_data.pdop, gps_data.hdop, gps_data.vdop, gps_data.epe
        ])

        time.sleep(READ_PERIOD / 1000.0)
        # Flush data to file
        csv_file.flush()

        # Wait for 100 ms
        time.sleep(READ_PERIOD/1000.0)



    input("Press key to exit\n") # Use raw_input() in Python 2
    ipcon.disconnect()