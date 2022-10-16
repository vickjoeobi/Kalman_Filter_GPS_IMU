#!/usr/bin/env python
# -*- coding: utf-8 -*-

HOST = "localhost"
PORT = 4223
UID = "XYZ" # Change XYZ to the UID of your GPS Bricklet 3.0

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_gps_v3 import BrickletGPSV3

if __name__ == "__main__":
    ipcon = IPConnection() # Create IP connection
    gps = BrickletGPSV3(UID, ipcon) # Create device object

    ipcon.connect(HOST, PORT) # Connect to brickd
    # Don't use device before ipcon is connected

    # Get current coordinates
    latitude, ns, longitude, ew = gps.get_coordinates()

    print("Latitude: " + str(latitude/1000000.0) + " °")
    print("N/S: " + ns)
    print("Longitude: " + str(longitude/1000000.0) + " °")
    print("E/W: " + ew)

    input("Press key to exit\n") # Use raw_input() in Python 2
    ipcon.disconnect()
