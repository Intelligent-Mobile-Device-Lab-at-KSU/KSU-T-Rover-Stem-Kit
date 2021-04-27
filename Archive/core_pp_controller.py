import sys # only needed for command line argument to validate MATLAB port to Python
import math

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

pose=[]
pose.append(float(sys.argv[1])) # robot x
pose.append(float(sys.argv[2])) # robot y
pose.append(float(sys.argv[3])) # angle w.r.t x-axis of global coordinate system
lx = float(sys.argv[4]) # goal point x
ly = float(sys.argv[5]) # goal point y
d = float(sys.argv[6]) # distance from robot to goal point
MaxTurnAngle = float(sys.argv[7]) # distance from robot to goal point

theta = pose[2] # car heading relative to world x-axis
beta = math.atan2( (ly-pose[1]), (lx-pose[0]) ) # direction in radians to goal point

if abs(theta-beta)<.000001:
    gamma = 0
else:
    gamma = theta - beta # direciton in radians to goal point in car's local coordinate where positive is right

x_offset = d*math.sin(gamma)*-1
y_offset = d*math.cos(gamma)
turnangle = (2*x_offset)/(d**2)

thesign  = mysign(  (math.sin(pose[2])*(lx-pose[0])) - (math.cos(pose[2])*(ly-pose[1])) )
turnangle = thesign*turnangle
# Ensure the turn control saturates at MaxTurnAngle defined by servo
if abs(turnangle) > MaxTurnAngle:
    turnangle = thesign*MaxTurnAngle

turnangle = myrem(turnangle,2*math.pi)
print(turnangle)
#print(pose[0])
#print(pose[1])
#print(pose[2])
#print(lx)
#print(ly)
#print(d)
#print(MaxTurnAngle)