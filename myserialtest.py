import threading
import serial
import time

ser = serial.Serial('/dev/ttyACM0', 9800, timeout=1)

maxTurnAngle=31 #in degrees
pwmTurnMin=1 #in Milliseconds
pwmTurnMid=1.5 #in Milliseconds
pwmTurnMax=2 #in Milliseconds
pwmThrottleMin=1 #in Milliseconds
pwmThrottleMid=1.5 #in Milliseconds
pwmThrottleMax=2 #in Milliseconds
pwmBaseFrequency=50.98 #in Hertz

dataIn = 'null'
pwmBitRes = 8
pwmTurn = 0
pwmSpeed = 0

#Set initial parameters on arduino
def txSettings():
    # output config legend: '<message_type(C for configure and D for controlls):bit_resolution:initial_turn_pwm_value:initial_speed_pwm_value:>'
    try:
        message = 'C:' + str(pwmBitRes) + ':' + str(pwmTurnMid) + ':' + str(pwmThrottleMid)
        ser.write(message.encode())
    except:
        print("ERROR: Could not send configs to Arduino")
        return -1

#Send control data
def txControls(): #Have one string fore setup and one for controls
        #output string legend: '<message_type(C for configure and D for controlls):turn_pwm_value:speed_pwm_value>'
        try:
            message = 'D:' + str(pwmTurn) + ':' + str(pwmSpeed)
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
        print("From Arduino: " + dataIn)
        time.sleep(.1)

def main():
    global dataIn
    print('BEGIN')
    th_arduino_serial = threading.Thread(name='serialLoop', target=serialLoop)
    th_arduino_serial.start()
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
    while True:
        txControls()
        while True:
            if (dataIn == 'ack'):
                dataIn = '0';
                break
            time.sleep(.1)
    print('Complete!')

main()