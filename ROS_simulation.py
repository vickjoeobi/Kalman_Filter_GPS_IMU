#!/usr/bin/env python

"""This script contains the full implementation of the ROS node in python. 
It subscribes to the IMU and GPS topics and saves the data in a csv file. 
The data is saved in the following format: time, latitude, longitude, 
altitude, speed, course, yaw_rate. 
The data is saved in the same folder 
as the script. The script is run by 
typing the following command in the terminal: rosrun sensor_handler simu.py"""

import rospy
import math
import csv
import datetime

import numpy as np
import matplotlib.dates as mdates
import matplotlib.pyplot as plt
from scipy.stats import norm
from sympy import Symbol, symbols, Matrix, sin, cos
from sympy import init_printing



from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from mavros_msgs.msg import GPSRAW


pub = rospy.Publisher('/mavros/test', Range, queue_size=1000) 
sen_num = 1
sensor_left = 0.0
sensor_bottom = 0.0

longi = []
lati = []
alti = []
speed = []
course = []
yaw_rate = []

def callback1(data):
    global orientation
    global angular_velocity
    global linear_acceleration

    orientation = data.orientation
    angular_velocity = data.angular_velocity
    linear_acceleration = data.linear_acceleration

    print ("Orientation")
    print ("IMU quaternion: ", data.orientation)
    print ("IMU angular velocity: ", data.angular_velocity)
    print ("IMU Linear acceleration: ", data.linear_acceleration)
    print("---------------")
    print("")
    
    return data.orientation, data.angular_velocity, data.linear_acceleration
    
    #print(sensor_left)

def callback2(data):
    global latitude
    global longitude
    global altitude

    latitude = data.lat
    longitude = data.lon
    altitude = data.alt

    print ("Location")
    print ("GPS Lat: ", data.lat)
    print ("GPS Lon: ", data.lon)
    print ("GPS Alt: ", data.alt)
    print("---------------")
    print("")

    print_data()

    return data.lat, data.lon, data.alt
    
def print_data():
    longi.append(longitude/10000000)
    lati.append(latitude/10000000)
    alti.append(altitude/1000)
    speed.append(math.sqrt(angular_velocity.x**2 + angular_velocity.y**2 + angular_velocity.z**2))
    course.append(math.atan2(angular_velocity.y, angular_velocity.x))
    yaw_rate.append(math.atan2(linear_acceleration.y, linear_acceleration.x))

def timer_callback(event):
    print("test")

def extended_kalman_filter():

    numstates = 5  # Number of states

    vs, psis, dts, xs, ys, lats, lons = symbols('v \psi T x y lat lon') # Symbols for states

    #We have different frequencies for the IMU and GPS
    dt = 1.0/50.0 # Sample Rate of the Measurements is 50Hz
    dtGPS=1.0/10.0 # Sample Rate of GPS is 10Hz

    vs, psis, dpsis, dts, xs, ys, lats, lons = symbols('v \psi \dot\psi T x y lat lon')

    # Dynamic model calculates how the states change over time
    gs = Matrix([[xs+(vs/dpsis)*(sin(psis+dpsis*dts)-sin(psis))],
                [ys+(vs/dpsis)*(-cos(psis+dpsis*dts)+cos(psis))],
                [psis+dpsis*dts],
                [vs],
                [dpsis]])

    state = Matrix([xs,ys,psis,vs,dpsis])

    # Initial Uncertainty in the states (P0)

    P = np.diag([1000.0, 1000.0, 1000.0, 1000.0, 1000.0])

    # Process Noise Covariance Matrix (Q)

    sGPS     = 0.5*8.8*dt**2  # assume 8.8m/s2 as maximum acceleration, forcing the vehicle
    sCourse  = 0.1*dt # assume 0.1rad/s as maximum turn rate for the vehicle
    sVelocity= 8.8*dt # assume 8.8m/s2 as maximum acceleration, forcing the vehicle
    sYaw     = 1.0*dt # assume 1.0rad/s2 as the maximum turn rate acceleration for the vehicle

    Q = np.diag([sGPS**2, sGPS**2, sCourse**2, sVelocity**2, sYaw**2])

    datafile = '/home/simsim/Desktop/CSV/data.csv'

    latitude, \
    longitude, \
    altitude,\
    speed, \
    course, \
    yawrate = np.loadtxt(datafile, delimiter=',', unpack=True,
                    skiprows=1)

    print('Read \'%s\' successfully.' % datafile)

    course =(-course+90.0)


    #Measurement function
    hs = Matrix([[xs],
                [ys],
                [vs],
                [dpsis]])

    #calculate the Jacobian of the measurement model
    JHs=hs.jacobian(state)

    # Measurement Noise Covariance Matrix (R)

    varGPS = 2.0 # Standard Deviation of GPS Measurement
    varspeed = 1.0 # Variance of the speed measurement
    varyaw = 0.1 # Variance of the yawrate measurement
    R = np.matrix([[varGPS**2, 0.0, 0.0, 0.0],
                [0.0, varGPS**2, 0.0, 0.0],
                [0.0, 0.0, varspeed**2, 0.0],
                [0.0, 0.0, 0.0, varyaw**2]])

    # Identity Matrix
    I = np.eye(numstates)

    # Approximate lon/lat to meters conversion to check location
    RadiusEarth = 6378388.0 # m
    arc = 2.0*np.pi*(RadiusEarth+altitude)/360.0 # m/°

    dx = arc * np.cos(latitude*np.pi/180.0) * np.hstack((0.0, np.diff(longitude))) # in m
    dy = arc * np.hstack((0.0, np.diff(latitude))) # in m

    mx = np.cumsum(dx)
    my = np.cumsum(dy)

    ds = np.sqrt(dx**2+dy**2)

    GPS = (ds!=0.0).astype('bool') # GPS Trigger for Kalman Filter

    # Initial State
    x = np.matrix([[mx[0], my[0], course[0]/180.0*np.pi, speed[0]/3.6+0.001, yawrate[0]/180.0*np.pi]]).T

    U = float(np.cos(x[2])*x[3])
    V = float(np.sin(x[2])*x[3])

    # Measurement Vector can also be denoted as Z
    measurements = np.vstack((mx, my, speed/3.6, yawrate/180.0*np.pi))

    m = measurements.shape[1]

    # Preallocation for Plotting
    x0 = []
    x1 = []
    x2 = []
    x3 = []
    x4 = []
    x5 = []
    Zx = []
    Zy = []
    Px = []
    Py = []
    Pdx= []
    Pdy= []
    Pddx=[]
    Pddy=[]
    Kx = []
    Ky = []
    Kdx= []
    Kdy= []
    Kddx=[]
    dstate=[]


    def savestates(x, Z, P, K):
        x0.append(float(x[0]))
        x1.append(float(x[1]))
        x2.append(float(x[2]))
        x3.append(float(x[3]))
        x4.append(float(x[4]))
        Zx.append(float(Z[0]))
        Zy.append(float(Z[1]))    
        Px.append(float(P[0,0]))
        Py.append(float(P[1,1]))
        Pdx.append(float(P[2,2]))
        Pdy.append(float(P[3,3]))
        Pddx.append(float(P[4,4]))
        Kx.append(float(K[0,0]))
        Ky.append(float(K[1,0]))
        Kdx.append(float(K[2,0]))
        Kdy.append(float(K[3,0]))
        Kddx.append(float(K[4,0]))
    longi = []
    lati = []
    nlongi = []
    nlati = []
    dlongi = []
    dlati = []

    for filterstep in range(m):

        # Time Update (Prediction)
        # ========================
        # Project the state ahead
        # see "Dynamic Matrix"

        longi.append(x[0])
        longiNum = float(x[0])
        lati.append(x[1])
        latiNum = float(x[1])
        if np.abs(yawrate[filterstep])<0.0001: # Driving straight
            x[0] = x[0] + x[3]*dt * np.cos(x[2])
            x[1] = x[1] + x[3]*dt * np.sin(x[2])
            x[2] = x[2]
            x[3] = x[3]
            x[4] = 0.0000001 # avoid numerical issues in Jacobians
            dstate.append(0)
        else: # otherwise
            x[0] = x[0] + (x[3]/x[4]) * (np.sin(x[4]*dt+x[2]) - np.sin(x[2]))
            x[1] = x[1] + (x[3]/x[4]) * (-np.cos(x[4]*dt+x[2])+ np.cos(x[2]))
            x[2] = (x[2] + x[4]*dt + np.pi) % (2.0*np.pi) - np.pi
            x[3] = x[3]
            x[4] = x[4]
            dstate.append(1)
        
        # Calculate the Jacobian of the Dynamic Matrix A
        # see "Calculate the Jacobian of the Dynamic Matrix with respect to the state vector"
        a13 = float((x[3]/x[4]) * (np.cos(x[4]*dt+x[2]) - np.cos(x[2])))
        a14 = float((1.0/x[4]) * (np.sin(x[4]*dt+x[2]) - np.sin(x[2])))
        a15 = float((dt*x[3]/x[4])*np.cos(x[4]*dt+x[2]) - (x[3]/x[4]**2)*(np.sin(x[4]*dt+x[2]) - np.sin(x[2])))
        a23 = float((x[3]/x[4]) * (np.sin(x[4]*dt+x[2]) - np.sin(x[2])))
        a24 = float((1.0/x[4]) * (-np.cos(x[4]*dt+x[2]) + np.cos(x[2])))
        a25 = float((dt*x[3]/x[4])*np.sin(x[4]*dt+x[2]) - (x[3]/x[4]**2)*(-np.cos(x[4]*dt+x[2]) + np.cos(x[2])))
        JA = np.matrix([[1.0, 0.0, a13, a14, a15],
                        [0.0, 1.0, a23, a24, a25],
                        [0.0, 0.0, 1.0, 0.0, dt],
                        [0.0, 0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 1.0]])
        
        
        # Project the error covariance ahead
        P = JA*P*JA.T + Q
        
        # Measurement Update (Correction)
        # ===============================
        # Measurement Function
        hx = np.matrix([[float(x[0])],
                        [float(x[1])],
                        [float(x[3])],
                        [float(x[4])]])

        if GPS[filterstep]: # with 10Hz, every 5th step
            JH = np.matrix([[1.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 1.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 1.0]])
        else: # every other step
            JH = np.matrix([[0.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 1.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 1.0]])
        
        S = JH*P*JH.T + R
        K = (P*JH.T) * np.linalg.inv(S)

        # Update the estimate via
        Z = measurements[:,filterstep].reshape(JH.shape[0],1)
        y = Z - (hx)                         # Innovation or Residual
        x = x + (K*y)

        # Update the error covariance
        P = (I - (K*JH))*P

        nlongi.append(x[0])
        nlati.append(x[1])
        dlongi.append(x[0] - longiNum)
        dlati.append(x[1] - latiNum)


        
        # Save states for Plotting
        savestates(x, Z, P, K)
    #convert longi, lati, nlongi, nlati as one csv file
    with open('/home/simsim/Desktop/CSV/output.csv', 'w', newline='') as csvfile:
        fieldnames = ['longitude', 'newLongitude', 'difference_Longitude', 'latitude', 'newLatitude', 'difference_Latitude']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for i in range(len(longi)):
            writer.writerow({'longitude': longi[i], 'newLongitude': nlongi[i], 'difference_Longitude': dlongi[i], 'latitude': lati[i], 'newLatitude': nlati[i], 'difference_Latitude': dlati[i]})


    def plotP():
        fig = plt.figure(figsize=(16,9))
        plt.semilogy(range(m),Px, label='$x$')
        plt.step(range(m),Py, label='$y$')
        plt.step(range(m),Pdx, label='$\psi$')
        plt.step(range(m),Pdy, label='$v$')
        plt.step(range(m),Pddx, label='$\dot \psi$')

        plt.xlabel('Filter Step')
        plt.ylabel('')
        plt.title('Uncertainty (Elements from Matrix $P$)')
        plt.legend(loc='best',prop={'size':22})
        #save the figure
        fig.savefig('/home/simsim/Desktop/CSV/uncertainty.png', bbox_inches='tight')

    plotP()

    #Kalman gain
    fig = plt.figure(figsize=(16,9))
    plt.step(range(len(measurements[0])),Kx, label='$x$')
    plt.step(range(len(measurements[0])),Ky, label='$y$')
    plt.step(range(len(measurements[0])),Kdx, label='$\psi$')
    plt.step(range(len(measurements[0])),Kdy, label='$v$')
    plt.step(range(len(measurements[0])),Kddx, label='$\dot \psi$')


    plt.xlabel('Filter Step')
    plt.ylabel('')
    plt.title('Kalman Gain (the lower, the more the measurement fullfill the prediction)')
    plt.legend(prop={'size':18})
    plt.ylim([-0.1,0.1]);
    #save the figure
    fig.savefig('/home/simsim/Desktop/CSV/kalman_gain.png', bbox_inches='tight')

    #state vector
    def plotx():
        fig = plt.figure(figsize=(16,16))

        plt.subplot(411)
        plt.step(range(len(measurements[0])),x0-mx[0], label='$x$')
        plt.step(range(len(measurements[0])),x1-my[0], label='$y$')

        plt.title('Extended Kalman Filter State Estimates (State Vector $x$)')
        plt.legend(loc='best',prop={'size':22})
        plt.ylabel('Position (relative to start) [m]')

        plt.subplot(412)
        plt.step(range(len(measurements[0])),x2, label='$\psi$')
        plt.step(range(len(measurements[0])),(course/180.0*np.pi+np.pi)%(2.0*np.pi) - np.pi, label='$\psi$ (from GPS as reference)')
        plt.ylabel('Course')
        plt.legend(loc='best',prop={'size':16})

        plt.subplot(413)
        plt.step(range(len(measurements[0])),x3, label='$v$')
        plt.step(range(len(measurements[0])),speed/3.6, label='$v$ (from GPS as reference)')
        plt.ylabel('Velocity')
        plt.ylim([0, 30])
        plt.legend(loc='best',prop={'size':16})

        plt.subplot(414)
        plt.step(range(len(measurements[0])),x4, label='$\dot \psi$')
        plt.step(range(len(measurements[0])),yawrate/180.0*np.pi, label='$\dot \psi$ (from IMU as reference)')
        plt.ylabel('Yaw Rate')
        plt.ylim([-0.6, 0.6])
        plt.legend(loc='best',prop={'size':16})
        plt.xlabel('Filter Step')

        plt.savefig('/home/simsim/Desktop/CSV/Extended-Kalman-Filter-CTRV-State-Estimates.png', bbox_inches='tight')

    plotx()

    def plotxy():

        fig = plt.figure(figsize=(16,9))

        # EKF State
        plt.quiver(x0,x1,np.cos(x2), np.sin(x2), color='#94C600', units='xy', width=0.05, scale=0.5)
        plt.plot(x0,x1, label='EKF Position', c='k', lw=5)

        # Measurements
        plt.scatter(mx[::5],my[::5], s=50, label='GPS Measurements', marker='+')
        #cbar=plt.colorbar(ticks=np.arange(20))
        #cbar.ax.set_ylabel(u'EPE', rotation=270)
        #cbar.ax.set_xlabel(u'm')

        # Start/Goal
        plt.scatter(x0[0],x1[0], s=60, label='Start', c='g')
        plt.scatter(x0[-1],x1[-1], s=60, label='Goal', c='r')

        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Position')
        plt.legend(loc='best')
        plt.axis('equal')
        plt.tight_layout()

        plt.savefig('/home/simsim/Desktop/CSV/Extended-Kalman-Filter-CTRV-Position.png', bbox_inches='tight')

    plotxy()
    
    print("Success!")

    


def listener():
     
    rospy.init_node('sensor_handler', anonymous=True)

    rospy.Subscriber('mavros/imu/data', Imu, callback1)
    rospy.Subscriber('mavros/gpsstatus/gps1/raw', GPSRAW, callback2)
    
    #timer = rospy.Timer(rospy.Duration(0.01), timer_callback)
    #convert to gps coordinates to csv
    

    rospy.spin()
    
    #timer.shutdown()

if __name__ == '__main__':
    print ("Running")
    listener()
    #add date and time to file name
    #filename = "/home/simsim/Desktop/CSV/"+"flight_data_" + str(datetime.datetime.now().strftime("%Y%m%d-%H-%M-%S")) +".csv"
    filename = "/home/simsim/Desktop/CSV/data.csv"
    csv_file = open(filename, 'w')
    
    writer = csv.writer(csv_file)
    writer.writerow(['longitude', 'latitude', 'altitude', 'speed', 'course', 'yaw_rate'])
    for i in range(len(longi)):
        writer.writerow([longi[i], lati[i], alti[i], speed[i], course[i], yaw_rate[i]])
    csv_file.close()

    print("Data written to csv file")

    #run kalman filter
    extended_kalman_filter()

    print("Kalman filter done")