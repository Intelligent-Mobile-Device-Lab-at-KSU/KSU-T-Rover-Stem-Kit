import socket
import signal
import sys
import pynmea2
import time
import io

# This code runs in Termux on the smartphone
print('Reading Configuration File: termux_trover_conf.txt ...')
fconf = open("termux_trover_conf.txt", "r")
Lines = fconf.readlines()
count=1
# Strips the newline character
for line in Lines:
    sarr=line.strip().replace(" ", "").split("=")
    if len(sarr[0])==0:
        continue
    #print(sarr)
    if count==1:
        thisRpiIP = sarr[1]
    count+=1
fconf.close()
print('Config loaded!')

destIP_udp = thisRpiIP # Change this to the IP address of the RPi on the smartphone hotspot
destPort_udp = 20001 # Do not change

s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
s.connect(('192.0.0.2',1236)) # Do not change

def signal_handler(sig,frame):
    s.close()
    print('done')
    sys.exit(0)

signal.signal(signal.SIGINT,signal_handler)
ss=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

seenFirstPosition = False
p1 = (0, 0)
p2 = (0,0)
gga=False
rmc=False 
o=''
while True:
    data=s.recv(115200)
    sdata=data.decode('ascii')
    buf = io.StringIO(sdata)
    nmea_sentence = '-------'
    while len(nmea_sentence)>0:
        nmea_sentence = buf.readline()
        if 'GGA' in nmea_sentence:
            msg_latlon=pynmea2.parse(nmea_sentence)
            o+="%s,%s"%(msg_latlon.latitude,msg_latlon.longitude)
            gga=True
        if 'RMC' in nmea_sentence:
            msg = pynmea2.parse(nmea_sentence)
            try:
                angle = float(msg.true_course)
                angle = 360+(90-angle)
                if angle > 360:
                    angle = angle - 360
            except:
                angle='None'
            o+="%s,"%(str(angle))
            rmc=True
                                                 
        if gga and rmc:
            #print(o)
            gga=False
            rmc=False 
            ss.sendto(o.encode(),(destIP_udp,destPort_udp))
            o=''
    time.sleep(.1)