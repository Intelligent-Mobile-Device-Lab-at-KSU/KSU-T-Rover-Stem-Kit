===termuxTrover.py===
runs in termux on phone
gps is connected over usbc directly to phone
phone hospot sends turn commands to rpi over udp
rpi is connected to phone hotspot

localPP = 1 # 0=remote PP calls to remote server, 1= run PP natively 

rpiIP = "192.168.142.203" #rpi IP on hotspot

fname = "rtkwaypoints.txt" # waypoint file

L = 3  # PP look ahead distance

logging.basicConfig(filename='app.log', filemode='w', format='%(message)s', level=logging.INFO)

rpiPort = 40000 # udp port rpi is receiving turn commands


===termuxRpiServo.py===
receives datagrams from phone, sends turn command to servo



