import netifaces as ni
import time
import socket
import sys
from gpiozero import AngularServo

servo = AngularServo(26, min_angle=-45, max_angle=45)

ni.ifaddresses('wlan0')
localIP = ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']

# Create a datagram socket
udp_server = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
udp_server.bind((localIP, 40000))

def signal_handler(sig,frame):
    print('done\n')
    sys.exit(0)

c = 1
while True:
    bytesAddressPair = udp_server.recvfrom(1024)
    message = bytesAddressPair[0]
    sdata = message.decode('utf-8')
    servo.angle = float(sdata)
    if (c % 10) == 0:
        c = 0
        print(sdata)
    c += 1
    time.sleep(.05)