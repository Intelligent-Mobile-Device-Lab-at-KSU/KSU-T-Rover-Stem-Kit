import socket
import signal
import sys
import pynmea2
import time
import io
import math
                            
destIP_udp = '192.168.212.203'
destPort_udp = 20001

s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
s.connect(('192.0.0.2',1236))

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
    time.sleep(.01)
    
