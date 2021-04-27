#!/usr/bin/env python3
import threading
import netifaces as ni
import numpy as np
import math
import time
import socket
import signal
from gpiozero import AngularServo

servo = AngularServo(26, min_angle=-45, max_angle=45)
waypoints_file = "....txt"  # Text File with GPS waypoints

###############################
# Pure Pursuit Config#
###############################
# Mapping and localization
waypoints = []
waypoints_utm = []

# Pure Pursuit Variables
L = 3  # meters
goalRadius = 3;  # meters
spacingBetweenCoarseWaypoints = 0.05  # 6 inches
pp_MaxTurnAngle = np.radians(14.5)  # degrees to avoid too large a PWM value
MaxAngularVelocity = math.pi / 8;  # radians per second; (not implemented yet need to implement and optimize to reduce fast angular changes)

###############################
# Wi-Fi Hotspot Connection from Phone to RPi#
###############################
# UDP from phone
localPort_gps = 20001  # The RPi will open this port for receiving GPS and sensor input from phone.
bufferSize = 1024

sensorDict = {}
sensorDict["compass"] = 90 # default

######################
# Getting GPS and Sensor from Phone
######################
# Automatically get RPIs IP address
localIP = ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']

# Create a datagram socket
UDPServerSocket_gps = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPServerSocket_gps.bind((localIP, localPort_gps))

# This function is called in a separate thread for listening
# for incoming datagrams from the phone.
def udpListener_gps(sensorDict):
    while True:
        bytesAddressPair = UDPServerSocket_gps.recvfrom(bufferSize)
        message = bytesAddressPair[0]
        sdata = message.decode('utf-8').split(',')
        sensorDict["gps"] = [float(sdata[1]), float(sdata[2])]
        if sdata[0] == 'None':
            g = 1  # default ignore, compass field not available until T-Rover moves
        else:
            sensorDict["compass"] = float(sdata[0])
        time.sleep(.05)

######################
# Pure Pursuit Controller
######################

# mysign - returns the sign of the variable x
# (x should be a number!)
def mysign(x):
    if x < 0:
        return -1
    if x == 0:
        return 0
    if x > 0:
        return 1

# myrem - returns the remainder of the variable x % y
# (x and y should be a number!)
def myrem(x, y):
    w = 0
    if x / y < 0:
        w = math.floor(x / y) + 1
    else:
        w = math.floor(x / y)
    return x - y * w

# purePursuit - This is the core controller of T-Rover
# Don't change anything here unless you know what you are doing!
def purePursuit(pose, lx, ly, d):
    speedval = 1
    # local variables
    theta = pose[2]  # car heading relative to world x-axis (i.e., Magnetic East)
    beta = math.atan2((ly - pose[1]), (lx - pose[0]))  # direction in radians to goal point

    if abs(theta - beta) < .000001:
        gamma = 0
    else:
        gamma = theta - beta  # direction in radians to goal point in car's local coordinate where positive is right

    x_offset = d * math.sin(gamma) * -1
    y_offset = d * math.cos(gamma)
    turnangle = (2 * x_offset) / (d ** 2)

    thesign = mysign((math.sin(pose[2]) * (lx - pose[0])) - (math.cos(pose[2]) * (ly - pose[1])))
    turnangle = thesign * turnangle

    # Ensure the turn control saturates at MaxTurnAngle defined by servo
    if abs(turnangle) > pp_MaxTurnAngle:
        turnangle = thesign * pp_MaxTurnAngle

    turnangle = myrem(turnangle, 2 * math.pi)
    return turnangle, speedval


# deg2utm - converts GPS lat, lon (spherical coordinates) to
# utm (cartesian coordinates)
# The output is the x and y position for a specific utmzone
def deg2utm(Lat, Lon):
    # Memory pre-allocation
    x = []
    y = []
    utmzone = []
    # Main Loop
    #
    la = Lat
    lo = Lon
    sa = 6378137.000000
    sb = 6356752.314245

    # e = ( ( ( sa ** 2 ) - ( sb ** 2 ) ) ** 0.5 ) / sa;
    e2 = (((sa ** 2) - (sb ** 2)) ** 0.5) / sb
    e2cuadrada = e2 ** 2
    c = (sa ** 2) / sb
    # alpha = ( sa - sb ) / sa;             #f
    # ablandamiento = 1 / alpha;   # 1/f
    lat = la * (math.pi / 180)
    lon = lo * (math.pi / 180)
    Huso = np.fix((lo / 6) + 31)
    S = ((Huso * 6) - 183)
    deltaS = lon - (S * (math.pi / 180))
    Letra = ''
    if (la < -72):
        Letra = 'C'
    elif (la < -64):
        Letra = 'D'
    elif (la < -56):
        Letra = 'E'
    elif (la < -48):
        Letra = 'F'
    elif (la < -40):
        Letra = 'G'
    elif (la < -32):
        Letra = 'H'
    elif (la < -24):
        Letra = 'J'
    elif (la < -16):
        Letra = 'K'
    elif (la < -8):
        Letra = 'L'
    elif (la < 0):
        Letra = 'M'
    elif (la < 8):
        Letra = 'N'
    elif (la < 16):
        Letra = 'P'
    elif (la < 24):
        Letra = 'Q'
    elif (la < 32):
        Letra = 'R'
    elif (la < 40):
        Letra = 'S'
    elif (la < 48):
        Letra = 'T'
    elif (la < 56):
        Letra = 'U'
    elif (la < 64):
        Letra = 'V'
    elif (la < 72):
        Letra = 'W'
    else:
        Letra = 'X'

    a = math.cos(lat) * math.sin(deltaS)
    epsilon = 0.5 * math.log((1 + a) / (1 - a))
    nu = math.atan(math.tan(lat) / math.cos(deltaS)) - lat
    v = (c / ((1 + (e2cuadrada * (math.cos(lat)) ** 2))) ** 0.5) * 0.9996
    ta = (e2cuadrada / 2) * epsilon ** 2 * (math.cos(lat)) ** 2
    a1 = math.sin(2 * lat)
    a2 = a1 * (math.cos(lat)) ** 2
    j2 = lat + (a1 / 2)
    j4 = ((3 * j2) + a2) / 4
    j6 = ((5 * j4) + (a2 * (math.cos(lat)) ** 2)) / 3
    alfa = (3 / 4) * e2cuadrada
    beta = (5 / 3) * alfa ** 2
    gama = (35 / 27) * alfa ** 3
    bm = 0.9996 * c * (lat - alfa * j2 + beta * j4 - gama * j6)
    xx = epsilon * v * (1 + (ta / 3)) + 500000
    yy = nu * v * (1 + ta) + bm
    if yy < 0:
        yy = 9999999 + yy
    x = xx
    y = yy
    utmzone = "%02d %c" % (Huso, Letra)
    return x, y, utmzone

# smoothWaypoints - smooth utm waypoints for T-Rover to follow.
# This function takes in the coarse utm waypoints and outputs
# linearly interpolated waypoints which improves T-Rover performance.
def smoothWaypoints(wp_utm, spacing):
    la = wp_utm[0:, 0]
    la = la.tolist()
    lo = wp_utm[0:, 1]
    lo = lo.tolist()
    utmz = wp_utm[1, 2]
    wla = [];
    wlo = [];
    u = [];
    for i in range(len(la) - 1):
        x2 = float(la[i + 1])
        y2 = float(lo[i + 1])
        x1 = float(la[i])
        y1 = float(lo[i])
        w1 = np.array([[x2], [y2]])
        wi = np.array([[x1], [y1]])
        v = w1 - wi;
        if np.linalg.norm(v) == 0:
            v = .000000000000000001
        d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2);
        num_points_that_fit = math.ceil(d / spacing);
        vd = (v / np.linalg.norm(v)) * spacing;
        for k in range(num_points_that_fit):
            wla.append((wi[0] + vd[0] * k));
            wlo.append((wi[1] + vd[1] * k));
            u.append(utmz);

    wla.append((float(la[len(la) - 1])))
    wlo.append((float(lo[len(lo) - 1])))
    u.append(utmz);
    return wla, wlo, u

# signal_handler - catches Ctrl+C gracefully.
def signal_handler(sig, frame):
    print('User ended T-Rover.\n')
    sys.exit(0)
    ######

# main - This is the main embedded system of T-Rover.
######
def main():
    print('T-Rover Initializing...')
    print(' ')
    ############START UDP###################
    print('Setting Up UDP Servers...')
    # Set up thread for UDP Server (phone is pushing as client to RPI)
    th_gps_udp = threading.Thread(name='udpListener_gps', target=udpListener_gps, args=(sensorDict,))
    th_gps_udp.start()

    print('Awaiting Valid GPS Signal...')
    # Check if sensorDict has gps value
    noGPS = True
    while noGPS:
        if 'gps' in sensorDict.keys():
            noGPS = False
        time.sleep(1)
    print('Valid GPS signal received from phone.')
    ##############END UDP#################
    print(' ')
    ##############START LOAD WAYPOINTS#################
    print('Loading Coarse GPS Waypoints...')
    # read from local waypoints_file and read into 2-D float array called: waypoints
    f1 = open(waypoints_file, "r")
    for x in f1:
        latLong = x.split(",");
        if ("\n" in latLong[1]):
            latLong[1] = latLong[1].replace("\n", "")
        latLong = [float(i) for i in latLong]
        waypoints.append(latLong)
    f1.close()

    print('Converting Coarse GPS Waypoints to UTM Coordinates')
    # convert all coarse gps waypoints (spherical) to utm coordinates (cartesian)
    for i in range(len(waypoints)):
        [txx, tyy, tuu] = deg2utm(waypoints[i][0], waypoints[i][1])
        waypoints_utm.append([txx, tyy, tuu])

    np_waypoints_utm = np.array(waypoints_utm)
    print('Smoothing UTM Waypoints...')
    # smooth coarse utm waypoints_utm
    [sxx, syy, suu] = smoothWaypoints(np_waypoints_utm, spacingBetweenCoarseWaypoints)
    troverGoal = (sxx[-1], syy[-1])
    troverGoal = np.array(troverGoal)
    print('Waypoints Loaded!')
    ##############END LOAD WAYPOINTS#################
    print(' ')
    print('T-Rover System Ready!')
    print('T-Rover Pure Pursuit Begin!')
    c = 1 # used for limiting rate of output to terminal
    distanceToGoal = 9999  # initial value
    utmzone = '' # initial value

    while (distanceToGoal > goalRadius):
        # Obtain robot location and orientation, load into pose
        rover_lat = sensorDict["gps"][0]  # gps lat
        rover_lon = sensorDict["gps"][1]  # gps long
        rover_heading_deg = sensorDict["compass"]  # heading angle from phone
        rover_heading_rad = float(np.radians(rover_heading_deg))
        [rover_x, rover_y, utmzone] = deg2utm(rover_lat, rover_lon)  # convert robot position from gps to utm
        pose = [rover_x, rover_y, rover_heading_rad]
        pose = np.array(pose)

        # Calculate distance to goal
        distanceToGoal = np.linalg.norm(pose[0:1] - troverGoal)

        # Find the next goal point within L (in utm coordinates)
        for i in range(len(sxx) - 1, -1, -1):
            goal_x = sxx[i]  # W[0]
            goal_y = syy[i]  # W[1]
            x2 = pose[0]
            y2 = pose[1]
            d = math.sqrt((goal_x - x2) ** 2 + (goal_y - y2) ** 2)
            if d <= L:
                break

        # Call pure pursuit and obtain turn angle
        [turnAngle_rad, speedValue] = purePursuit(pose, goal_x, goal_y, d)
        turnAngle_deg = float(np.degrees(turnAngle_rad))
        servo.angle = -turnAngle_deg

        # Print out the turn angle every 10 turn degrees
        if (c % 10) == 0:
            print('Turn Angle (Deg): %f, D_Goal: %d' % (turnAngle_deg, d))
            c = 0
        c += 1

    print('Goal Reached!')

# Set-up Ctrl+C handler
signal.signal(signal.SIGINT, signal_handler)

# Call main
main()