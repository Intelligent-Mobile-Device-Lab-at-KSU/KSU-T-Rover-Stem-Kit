#!/usr/bin/env python3
import threading
import netifaces as ni
import numpy as np
import math
import time
import serial
import socket
import fcntl
import struct
import threading as th

##############
# PWM Declarations (PWM occurs on Arduino)#
##############
#General configs
maxTurnAngle=31 #in degrees
stopTurnPWM=1500 #in milliseconds
maxTurnPWM=2000
minTurnPWM=1000
ThrottleMin=900 #in Milliseconds
ThrottleStop=1500 #in Milliseconds
ThrottleMax=2000 #in Milliseconds

#default serial vals
turnPWM = stopTurnPWM
speed = ThrottleStop
nearStartingWaypoint = 0;
dataIn = 'null'

###############################
# Pure Pursuit Config#
###############################
# Mapping and localization
fname = "waypoints.txt"
waypoints = []
waypoints_utm = []

# Pure Pursuit Variables
L = 5 # meters
goalRadius = .5; # meters
spacingBetweenCoarseWaypoints =  0.05# 6 inches
pp_MaxTurnAngle = np.radians(14.5) #degrees to avoid too large a PWM value
MaxAngularVelocity = math.pi/8; # radians per second; (not implemented yet need to implement and optimize to reduce fast angular changes)

###############################
# Wi-Fi Hotspot Connection from Phone to RPi#
###############################

# UDP from phone
localPort = 20001  # The RPi will open this port for receiving GPS and sensor input from phone.
bufferSize = 1024

sensorDict = {}

###############################
# Serial connection for Arduino#
###############################

#Initialize serial connection
ser = serial.Serial('/dev/ttyACM0', 9800, timeout=1)

#Set initial parameters on arduino
def txSettings():
    global stopTurnPWM
    global ThrottleStop
    # output config legend: '<message_type(C for configure and D for controlls):initial_turn_angle:initial_speed>'
    try:
        message = 'C:' + str(int(stopTurnPWM)) + ':' + str(int(ThrottleStop))
        ser.write(message.encode())
    except:
        print("ERROR: Could not send configs to Arduino")
        return -1

#Send control data
def txControls(): #Have one string fore setup and one for controls
        global turnPWM
        global speed
        global nearStartingWaypoint
        #output string legend: '<message_type(C for configure and D for controlls):turn_angle:speed>'
        try:
            message = 'D:' + str(int(turnPWM)) + ':' + str(int(speed)) + ':' + str(int(nearStartingWaypoint))
            ser.write(message.encode())
        except:
            print('ERROR: Could not send controls to Arduino')
            return -1

#Run loop for sending and recieving serial data
def serialLoop():
    while True:
        global dataIn
        dat = ser.readline()
        dataIn = dat.decode("utf-8").rstrip()
        #print("From Arduino: " + dataIn)
        time.sleep(.1)

#set steering PWM value for arduino
def turnControl(angle): //assumes right turns are positive and left turns are negative
    global turnPWM
    turnPWM = (maxTurnPWM-minTurnPWM)*(((maxTurnAngle/2)-angle)/maxTurnAngle)+minTurnPWM
    

#set throttle PWM value for arduino
def throttleControl(spdPercent):
    global speed
    speed = (ThrottleMax-ThrottleMin)*spdPercent+ThrottleMin

######################
# Getting GPS and Sensor from Phone
######################
ni.ifaddresses('wlan0')
localIP = ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']
#localIP = get_ip_address('wlan0')#"192.168.101.203"#socket.gethostbyname(socket.gethostname())
# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# Bind to address and ip
UDPServerSocket.bind((localIP, localPort))

def udpListener2(sensorDict):
    # Listen for incoming datagrams

    # Each sensor has 3 values that are delimited by the ids 1, 2, 3, 4, 5, & 6
    # Not sure what the ids 2 & 6 correspond to, and no sensor has been delimited by 2

    # GPS data           | begins with 1 | (latitude, longitude, altitude)  transmitted once per second
    # accelerometer data | begins with 3 | (x,y,z)  m/s^2
    # gyroscope data     | begins with 4 | (x,y,z)  rad/s
    # magnetometer data  | begins with 5 | (x,y,z)  in microTeslas

    #valid_sensor_dict = {1.0: "gps", 2.0: "unknown", 3.0: "accel", 4.0: "gyro", 5.0: "mag", 6.0: "unknown"}

    while True:

        bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)

        message = bytesAddressPair[0]

        # bytes are delimited by commas, so we do a few things:
        # - split into a string array based on the commas
        # - remove extraneous whitespace at the beginning/end of each string
        # - remove the extraneous data at the beginning & end of the buffer
        sdata = message.decode('utf-8')
        data = sdata.split(',')#[item.strip() for item in str(message).split(',')][1:]
        if len(data) == 3:
            sensorDict["gps"] = [float(data[0]), float(data[1])]
        else:
            sensorDict["gps"] = [float(data[0]), float(data[1])]
            #print("MAG: %f, %f, %f" % (float(data[2]),float(data[3]),float(data[4])))
            angle = math.atan2(float(data[3]), float(data[4]))
            angle = math.degrees(angle) + 270
            if angle > 360:
                angle = angle - 360
            sensorDict["compass"] = angle
            #print(angle)
        #print(data[1])
        #data[-1] = data[-1][:-1]

        # convert the string array to float array
        #data = [float(item) for item in data]
        #sensorDict["gps"] = [data[0],data[1]]
        # based on https://www.geeksforgeeks.org/python-split-list-into-lists-by-particular-value/
        # loop through & split data into sub-arrays based on their corresponding sensor id (i.e. 1.0, 2.0, 3.0, etc.)
        #size = len(data)
        #idx_list = [idx for idx, val in enumerate(data) if val in valid_sensor_dict.keys()]
        #splitData = [data[i: j] for i, j in zip([0] + idx_list, idx_list + ([size] if idx_list[-1] != size else []))][
        #            1:]

        # convert array to dictionary based on the sensor

        #for values in splitData:
        #    sensorNumber = values[0]
        #    sensorDict[valid_sensor_dict[sensorNumber]] = values[1:]

        # remove unknown sensor data from dict
        #if "unknown" in sensorDict:
        #    sensorDict.pop("unknown")

        # calculates magnetic north from magnetometer readings if available
        # phone MUST be oriented in landscape with volume buttons and power button facing pointed upwards
        #if "mag" in sensorDict:
            # sometimes the phone returns a magnetometer reading that doesn't have the proper length, this if statement
            # avoids that error
        #    if len(sensorDict["mag"]) == 3:
        #        mag = sensorDict["mag"]
                # returns angle in radians from -Pi to +Pi
        #        angle = math.atan2(mag[1], mag[2])
                # converts angle in radians to degrees from 0 to 360
                # converts magnetic north as being 360/0 degrees to magnetic east being 360/0 degrees
                # if magnetic north is desired as 360/0, change the number added to angle to 180 rather than 270
        #        angle = math.degrees(angle) + 270
        #        if angle > 360:
        #            angle = angle - 360
        #        sensorDict["compass"] = angle
        time.sleep(.1)

def udpListener(sensorDict):
    # Listen for incoming datagrams

    # Each sensor has 3 values that are delimited by the ids 1, 2, 3, 4, 5, & 6
    # Not sure what the ids 2 & 6 correspond to, and no sensor has been delimited by 2

    # GPS data           | begins with 1 | (latitude, longitude, altitude)  transmitted once per second
    # accelerometer data | begins with 3 | (x,y,z)  m/s^2
    # gyroscope data     | begins with 4 | (x,y,z)  rad/s
    # magnetometer data  | begins with 5 | (x,y,z)  in microTeslas

    valid_sensor_dict = {1.0: "gps", 2.0: "unknown", 3.0: "accel", 4.0: "gyro", 5.0: "mag", 6.0: "unknown"}

    while True:

        bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)

        message = bytesAddressPair[0]

        # bytes are delimited by commas, so we do a few things:
        # - split into a string array based on the commas
        # - remove extraneous whitespace at the beginning/end of each string
        # - remove the extraneous data at the beginning & end of the buffer
        data = [item.strip() for item in str(message).split(',')][1:]
        data[-1] = data[-1][:-1]

        # convert the string array to float array
        data = [float(item) for item in data]

        # based on https://www.geeksforgeeks.org/python-split-list-into-lists-by-particular-value/
        # loop through & split data into sub-arrays based on their corresponding sensor id (i.e. 1.0, 2.0, 3.0, etc.)
        size = len(data)
        idx_list = [idx for idx, val in enumerate(data) if val in valid_sensor_dict.keys()]
        splitData = [data[i: j] for i, j in zip([0] + idx_list, idx_list + ([size] if idx_list[-1] != size else []))][
                    1:]

        # convert array to dictionary based on the sensor

        for values in splitData:
            sensorNumber = values[0]
            sensorDict[valid_sensor_dict[sensorNumber]] = values[1:]

        # remove unknown sensor data from dict
        if "unknown" in sensorDict:
            sensorDict.pop("unknown")

        # calculates magnetic north from magnetometer readings if available
        # phone MUST be oriented in landscape with volume buttons and power button facing pointed upwards
        if "mag" in sensorDict:
            # sometimes the phone returns a magnetometer reading that doesn't have the proper length, this if statement
            # avoids that error
            if len(sensorDict["mag"]) == 3:
                mag = sensorDict["mag"]
                # returns angle in radians from -Pi to +Pi
                angle = math.atan2(mag[1], mag[2])
                # converts angle in radians to degrees from 0 to 360
                # converts magnetic north as being 360/0 degrees to magnetic east being 360/0 degrees
                # if magnetic north is desired as 360/0, change the number added to angle to 180 rather than 270
                angle = math.degrees(angle) + 270
                if angle > 360:
                    angle = angle - 360
                sensorDict["compass"] = angle
        time.sleep(.1)
######################
# Pure Pursuit Controller
######################

def mysign(x):
    if x<0:
        return -1
    if x==0:
        return 0
    if x>0:
        return 1

def myrem(x,y):
    w = 0
    if x/y < 0:
        w = math.floor(x/y) + 1
    else:
        w = math.floor(x/y)
    return x - y * w

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


# deg2utm - converts GPS lat, lon (spherical coordinates) to utm (cartesian coordinates)
# The output is the x and y position of the T-Rover for a specific utmzone
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

def smoothWaypoints(wp_utm, spacing):
    la = wp_utm[0:,0]
    la = la.tolist()
    lo = wp_utm[0:,1]
    lo = lo.tolist()
    utmz = wp_utm[1,2]
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

######
# Main#
######
def main():
    print('T-Rover Initializing...')

    print(' ')
    print('############START UDP###################')
    print('Setting Up UDP Server...')
    print(localIP)
    print(localPort)
    # Set up thread for UDP Server (phone is pushing as client to RPI)
    th_udp = threading.Thread(name='udpListener2', target=udpListener2, args=(sensorDict, ))
    th_udp.start()

    print('Awaiting Valid GPS Signal...')

    # Check if sensorDict has gps value
    noGPS = True
    while noGPS:
        if 'gps' in sensorDict.keys():
            noGPS = False
        time.sleep(1)
    print('Valid GPS signal received from phone.')
    print('##############END UDP#################')
    print(' ')

    print(' ')
    print('#############START SERIAL##################')
    global dataIn
    th_arduino_serial = threading.Thread(name='serialLoop', target=serialLoop)
    th_arduino_serial.start()
    print('Awaiting: hello?, from Arduino...')
    while True:
        if (dataIn == 'hello?'):
            break
        time.sleep(1)
    print('Arduino communication established.')
    print('Sending Config Settings to Arduino.')
    txSettings()
    while True:
        if (dataIn == 'thank you!'):
            break
        time.sleep(1)
    print('Arduino configuration complete.')
    print('#############END SERIAL##################')
    print(' ')
    print('##############START WAYPOINTS#################')
    print('Loading Coarse GPS Waypoints...')
    # read from local file "waypoints.txt" and read into 2-D float array called: waypoints.txt
    f3 = open(fname, "r")
    for x in f3:
        latLong = x.split(",");
        if ("\n" in latLong[1]):
            latLong[1] = latLong[1].replace("\n", "")
        latLong = [float(i) for i in latLong]
        waypoints.append(latLong)
    f3.close()

    print('Converting Coarse GPS Waypoints to UTM Coordinates')
    # convert all coarse gps waypoints.txt to utm coordinates
    for i in range(len(waypoints)):
        [txx, tyy, tuu] = deg2utm(waypoints[i][0], waypoints[i][1])
        waypoints_utm.append([txx, tyy, tuu])

    np_waypoints_utm = np.array(waypoints_utm)
    print('Smoothing UTM Waypoints...')
    # smooth coarse utm waypoints.txt
    [sxx, syy, suu] = smoothWaypoints(np_waypoints_utm, spacingBetweenCoarseWaypoints)
    troverGoal = (sxx[-1], syy[-1])
    troverGoal = np.array(troverGoal)
    print('##############END WAYPOINTS#################')
    print('')
    print('##############START INIT POSITION#################')
    print('Please place T-Rover near the first waypoint...')
    global nearStartingWaypoint
    while True:
        rover_lat = sensorDict["gps"][0]  # gps lat
        rover_lon = sensorDict["gps"][1]  # gps long
        #rover_lat = 33.830619
        #rover_lon = -84.587818
        [rover_x, rover_y, utmzone] = deg2utm(rover_lat, rover_lon)
        theRange = range(len(sxx) - 1, -1, -1)
        goal_x = sxx[0]  # W[0]
        goal_y = syy[0]  # W[1]
        x2 = rover_x
        y2 = rover_y
        d = math.sqrt((goal_x - x2) ** 2 + (goal_y - y2) ** 2)
        if d <= L:
            nearStartingWaypoint = 1
            break
        txControls()
        time.sleep(1)
    print('T-Rover is Near Initial Waypoint!')
    print('##############END INIT POSITION#################')
    print('')
    print(' ')
    print('T-Rover System Ready!')
    print('T-Rover Pure Pursuit Begin!')
    distanceToGoal = 9999 # initial value
    utmzone = ''
    while (distanceToGoal > goalRadius):
        noPose = True
        while noPose:
            if sensorDict["gps"][0] != 'null':
                noPose = False
            time.sleep(.1)
        rover_lat = sensorDict["gps"][0] #gps lat
        rover_lon = sensorDict["gps"][1] #gps long
        #test init gps 33.830619, -84.587818
        #rover_lat = 33.830619
        #rover_lon = -84.587818
        rover_heading_deg = sensorDict["compass"] # bearing angle (we may need to smooth this)
        #print(rover_heading_deg)
        sensorDict["gps"][0] = 'null'
        sensorDict["gps"][1] = 'null'
        sensorDict["compass"] = 'null'
        rover_heading_rad = float(np.radians(rover_heading_deg))
        [rover_x, rover_y, utmzone] = deg2utm(rover_lat, rover_lon) # convert robot position from gps to utm
        pose = [rover_x, rover_y, rover_heading_rad]

        #print('Current pose: %f, %f, %f' % (pose[0],pose[1],pose[2]))
        pose = np.array(pose)
        #print('Current pose: %f, %f, %f' % (pose[0], pose[1], pose[2]))
        #print(pose)
        # Calculate distance to goal
        distanceToGoal = np.linalg.norm(pose[0:1] - troverGoal)
        #print('Distance to goal: %d' % (distanceToGoal))
        # Calculate goal point in utm coordinates
        for i in range(len(sxx)-1,-1,-1):
            #W = wp_utm_smooth[i]
            goal_x = sxx[i]#W[0]
            goal_y = syy[i]#W[1]
            x2 = pose[0]
            y2 = pose[1]
            d = math.sqrt((goal_x - x2)**2 + (goal_y - y2)**2)
            if d <= L:
                break
        #print('Goal_X: %d, Goal_Y: %d' % (goal_x, goal_y))
        [turnAngle_rad, speedValue] = purePursuit(pose, goal_x, goal_y, d)
        turnAngle_deg = float(np.degrees(turnAngle_rad))
        print('Turn Angle (Deg): %f, Speed: %d' % (turnAngle_deg, speedValue))
        turnControl(turnAngle_deg)
        throttleControl(speedValue)
        txControls()
        while True:
            if dataIn.find("ack") != -1:
                xx=dataIn.split()
                print(xx[1])
                dataIn = '0'
                break
            time.sleep(.1)
    print('Goal Reached!')

main()
