#!/usr/bin/env python3
import threading
import netifaces as ni
import numpy as np
import math
import time
import socket
import signal
import io
import sys
import pynmea2
matlabIPaddress = "192.168.8.118"
print('Reading Boat ESP32 Configuration File: conf.txt ...')
fconf = open("conf.txt", "r")
Lines = fconf.readlines()
count=1
# Strips the newline character
for line in Lines:
    sarr=line.strip().replace(" ", "").split("=")
    if len(sarr[0])==0:
        continue
    #print(sarr)
    if count==1:
        thisL = int(sarr[1])
    elif count==2:
        esp32localIPaddress = sarr[1]
    elif count==3:
        thiswaypointsfname = sarr[1]
    elif count==4:
        thisWebSwitchIP = sarr[1]
    count+=1
fconf.close()
print('Config loaded!')

waypoints_file = thiswaypointsfname  # Text File with GPS waypoints Lat, Long

####
#WebSwitch: Autonomous/Manual
###
driveMode = 0 # manual steering using controller
def fetchDriveMode():
    global driveMode
    msgFromClient       = "0"
    bytesToSend         = str.encode(msgFromClient)
    serverAddressPort   = (thisWebSwitchIP, 11000)
    bufferSize          = 1024 
    # Create a UDP socket at client side
    UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    
    # Send to server using created UDP socket
    UDPClientSocket.settimeout(1)
    while(True):
        try:
            UDPClientSocket.sendto(bytesToSend, serverAddressPort)
            msgFromServer = UDPClientSocket.recvfrom(bufferSize)
        except:
            continue
        driveMode = int(msgFromServer[0])
        time.sleep(.5)
        #msg = "Message from Server {}".format(msgFromServer[0])
        #print(msg)

###############################
# Pure Pursuit Config#
###############################
# Mapping and localization
waypoints = []
waypoints_utm = []
spacingBetweenCoarseWaypoints = 0.05  # 6 inches

# Pure Pursuit Variables
L = thisL  # meters
goalRadius = 1  # meters
pp_MaxTurnAngle = np.radians(14.5)  # degrees to avoid too large a PWM value

# GPS TCP Server Port: The UART bridge app should convert serial to TCP Server 
localPort_gps = 20001  # Termux will connect to this TCP port for receiving GPS from GPS-RTK board via UART bridge app.
bufferSize = 1024

# GPS Dictionary
sensorDict = {}
sensorDict["compass"] = 90 # default

######################
# Getting GPS
######################
# Automatically get Termux IP address
localIP = ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']
print(localIP)
# Create a GPS TCP client
TCPClientSocket_gps = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
TCPClientSocket_gps.connect((localIP, localPort_gps)) # Do not change

# This function is called in a separate thread for listening
# for incoming bytes streamed from the GPS-RTK breakout board via the UART bridge.
def tcpListener_gps(sensorDict):
    o = ''
    gga=False
    rmc=False
    while(True):
        #reads a line of data (from local port), it expects a line ending
        data=TCPClientSocket_gps.recv(115200)
        sdata=data.decode('ascii')
        buf = io.StringIO(sdata)
        nmea_sentence = '---------'
        while(len(nmea_sentence)>0):
            nmea_sentence = buf.readline()
            if 'GGA' in nmea_sentence:
                try:
                    msg_latlon=pynmea2.parse(nmea_sentence)
                    o+="%s,%s"%(msg_latlon.latitude,msg_latlon.longitude)
                    gga=True
                except:
                    g=1 #ignore if pynmea2 fails to parse
            if 'RMC' in nmea_sentence:
                try:
                    msg = pynmea2.parse(nmea_sentence)
                    angle = float(msg.true_course)
                    angle = 360+(90-angle)
                    if angle > 360:
                        angle = angle - 360
                except:
                    angle='-1'
                o+="%s,"%(str(angle))
                rmc=True
                                                 
            if gga and rmc:
                sdata = o.split(",")
                sensorDict["gps"] = [float(sdata[1]), float(sdata[2])]
                if sdata[0] == 'None':
                    g = 1  # default ignore, compass field not available until T-Rover moves
                else:
                    sensorDict["compass"] = float(sdata[0])
                o = ''
                gga=False
                rmc=False
        time.sleep(.1)

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
    wla = []
    wlo = []
    u = []
    for i in range(len(la) - 1):
        x2 = float(la[i + 1])
        y2 = float(lo[i + 1])
        x1 = float(la[i])
        y1 = float(lo[i])

        w1 = np.array([[x2], [y2]])
        wi = np.array([[x1], [y1]])
        v = w1 - wi

        if np.linalg.norm(v) == 0:
            v = .000000000000000001
        d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        num_points_that_fit = math.ceil(d / spacing)
        vd = (v / np.linalg.norm(v)) * spacing
        for k in range(int(num_points_that_fit)):
            wla.append((wi[0] + vd[0] * k))
            wlo.append((wi[1] + vd[1] * k))
            u.append(utmz)

    wla.append((float(la[len(la) - 1])))
    wlo.append((float(lo[len(lo) - 1])))
    u.append(utmz)
    return wla, wlo, u

# signal_handler - catches Ctrl+C gracefully.
def signal_handler(sig, frame):
    TCPClientSocket_gps.close()
    print('User ended KSU-Boat.\n')
    sys.exit(0)
    ######

# Set-up Ctrl+C handler
signal.signal(signal.SIGINT, signal_handler)
def main():
    # main - This is the main embedded system of T-Rover.
    ######
    print('KSU-Boat Initializing...')
    print(' ')
    print('Setting Up Remote Web Switch (check for manual/autonomous)')
    th_updateDriveMode = threading.Thread(name='fetchDriveMode', target=fetchDriveMode)
    th_updateDriveMode.start()
    ############START TCP GPS Client Init###################
    print('Setting Up Termux TCP Client for incoming GPS...')
    # Set up thread for Termux TCP Client (GPS-RTK is pushing GPS over serial cable, UART bridge app converting to TCP server, Termux connects as TCP client from this script)
    th_gps_tcp = threading.Thread(name='tcpListener_gps', target=tcpListener_gps, args=(sensorDict,))
    th_gps_tcp.start()

    print('Awaiting Valid GPS Signal...')
    # Check if sensorDict has gps value
    noGPS = True
    while noGPS:
        print(sensorDict)
        if 'gps' in sensorDict.keys():
            noGPS = False
        time.sleep(1)
    print('Valid GPS signal received from phone.')
    ##############END TCP GPS Client Init#################
    print(' ')
    ##############START LOAD WAYPOINTS#################
    print('Loading Coarse GPS Waypoints...')
    # read from local waypoints_file and read into 2-D float array called: waypoints
    print(waypoints_file)
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
    print('Attempting to contact ESP32...')
    # TO-DO
    # Create a temporary datagram socket listener to hear from ESP32
    UDPClientSocket_esp32 = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    print("UDP client created.")
    esp32IPaddress = (esp32localIPaddress,5000)
    while(True):
        # Attempt to contact ESP32
        UDPClientSocket_esp32.settimeout(1) # if no response within 1 second throw an exception
        print("Sending HELLO...")
        UDPClientSocket_esp32.sendto(str.encode("HELLO"),esp32IPaddress)
        try:
            bytesAddressPair = UDPClientSocket_esp32.recvfrom(bufferSize)
            print("Received From ESP32: %s" % (bytesAddressPair[0].decode()[:2]))
            if bytesAddressPair[0].decode()[:2] == "OK":
                print("Success!")
                break
        except:
            continue # no response
    print('ESP32 Connected!')
    print(' ')
    print('KSU-Boat System Ready!')
    print('KSU-Boat Pure Pursuit Begin!')
    c = 1 # used for limiting rate of output to terminal
    distanceToGoal = 9999  # initial value
    utmzone = '' # initial value
    finished = False
    UDPClientSocket_Telemetry = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    while(True):
        if (driveMode==0): # driveMode == Manual Steering  
            print('Manual Steering Available!')
            cmmd="-100"
            UDPClientSocket_esp32.sendto(cmmd.encode(), esp32IPaddress)
            time.sleep(5);
        else:   # driveMode == Autonomous Steering          
            if (distanceToGoal > goalRadius):
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
                UDPClientSocket_esp32.sendto(str(-turnAngle_deg).encode(), esp32IPaddress)
                stelemetry = "%d,%s,%s" % (L,rover_lat,rover_lon)
                UDPClientSocket_Telemetry.sendto(stelemetry.encode(),(matlabIPaddress,4411))
                # Print out the turn angle every 10 turn degrees
                if (c % 10) == 0:
                    print('Turn Angle (Deg): %f, D_Goal: %d' % (turnAngle_deg, d))
                    c = 0
                c += 1
                time.sleep(.1)
            else:
                print('Goal Reached! Manual Steering Available!')
                while(True): # Change back to manual steering
                    cmmd="-100"
                    UDPClientSocket_esp32.sendto(cmmd.encode(), esp32IPaddress)
                    time.sleep(5);
                
# Call main
main()
