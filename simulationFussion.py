#!/usr/bin/env python

import rospy
import math

import numpy as np
import matplotlib.dates as mdates
import matplotlib.pyplot as plt
from scipy.stats import norm
from sympy import Symbol, symbols, Matrix, sin, cos
from sympy import init_printing
init_printing(use_latex=True)



from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from mavros_msgs.msg import GPSRAW


pub = rospy.Publisher('/mavros/test', Range, queue_size=1000) 
sen_num = 1
sensor_left = 0.0
sensor_bottom = 0.0

def callback1(data):
    print ("IMU quaternion: ", data.orientation)
    print ("IMU angular velocity: ", data.angular_velocity)
    print ("IMU Linear acceleration: ", data.linear_acceleration)
    print("---------------")
    print("")
    
    return data.orientation, data.angular_velocity, data.linear_acceleration
    
    #print(sensor_left)

def callback2(data):
    print ("GPS Lat: ", data.lat)
    print ("GPS Lon: ", data.lon)
    print ("GPS Alt: ", data.alt)
    print("---------------")
    print("")

    return data.lat, data.lon, data.alt

def timer_callback(event):
    print("test")
    


def listener():
     
    rospy.init_node('sensor_handler', anonymous=True)

    rospy.Subscriber('mavros/imu/data', Imu, callback1)
    rospy.Subscriber('mavros/gpsstatus/gps1/raw', GPSRAW, callback2)
    
    #timer = rospy.Timer(rospy.Duration(0.01), timer_callback)

    rospy.spin()    
    timer.shutdown()

def kalman_filter(yawrate, speed, course, latitude, longitude, altitude):

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

    yawrate = yawrate
    speed = speed
    course = course
    latitude = latitude
    longitude = longitude
    altitude = altitude


    #Measurement function
    hs = Matrix([[xs],
                [ys],
                [vs],
                [dpsis]])

    #calculate the Jacobian of the measurement model
    JHs=hs.jacobian(state)

    # Measurement Noise Covariance Matrix (R)

    varGPS = 6.0 # Standard Deviation of GPS Measurement
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

    while True:

        # Time Update (Prediction)
        # ========================
        # Project the state ahead
        # see "Dynamic Matrix"
        if np.abs(yawrate)<0.0001: # Driving straight
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

        if GPS: # with 10Hz, every 5th step
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
        Z = measurements.reshape(JH.shape[0],1)
        y = Z - (hx)                         # Innovation or Residual
        x = x + (K*y)

        # Update the error covariance
        P = (I - (K*JH))*P


        
        # Save states for Plotting
        savestates(x, Z, P, K)
        
        print("x: ", x[0], x[1], x[2], x[3], x[4])
        print("z: ", Z[0], Z[1], Z[2], Z[3])

        # Predict next state with the most current state and covariance matrix
        GPS = not GPS

if __name__ == '__main__':
    print ("Running")
    listener()
 
