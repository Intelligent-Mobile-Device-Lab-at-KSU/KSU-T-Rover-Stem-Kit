import socket
import math

localIP = socket.gethostbyname(socket.gethostname())

localPort = 20001

bufferSize = 1024

print(localIP)
print(localPort)

msgFromServer = "Hello UDP Client"

bytesToSend = str.encode(msgFromServer)

# Create a datagram socket

UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Bind to address and ip

UDPServerSocket.bind((localIP, localPort))

print("UDP server up and listening")

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
    splitData = [data[i: j] for i, j in zip([0] + idx_list, idx_list + ([size] if idx_list[-1] != size else []))][1:]

    # convert array to dictionary based on the sensor
    sensorDict = {}
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

    print(sensorDict)
    print('gps' in sensorDict.keys())
