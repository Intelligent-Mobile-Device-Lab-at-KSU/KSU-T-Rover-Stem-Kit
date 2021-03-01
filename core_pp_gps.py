import sys # only needed for command line argument to validate MATLAB port to Python
import numpy as np
import math

f1 = open('C:\\Users\\gtg498u\\Documents\\MATLAB\\trover\\thelat.txt', 'r')
thelat = f1.read()
f1.close()

f2 = open('C:\\Users\\gtg498u\\Documents\\MATLAB\\trover\\thelon.txt', 'r')
thelon = f2.read()
f2.close()

thelat = thelat.split(",");
thelon = thelon.split(",");
#print(len(thelat))
#print(len(thelon))

thelatt = [float(i) for i in thelat];
thelonn = [float(i) for i in thelon];
#print(thelatt[0]+100)
#print(thelonn[0]+100)
n1=len(thelatt);
n2=len(thelonn);
if (n1!=n2):
    print('Lat and Lon vectors should have the same length');

def deg2utm(Lat,Lon):
    # Memory pre-allocation
    x=[]
    y=[]
    utmzone = [];
    # Main Loop
    #
    la=Lat;
    lo=Lon;
    sa = 6378137.000000
    sb = 6356752.314245

    #e = ( ( ( sa ** 2 ) - ( sb ** 2 ) ) ** 0.5 ) / sa;
    e2 = ( ( ( sa ** 2 ) - ( sb ** 2 ) ) ** 0.5 ) / sb;
    e2cuadrada = e2 ** 2;
    c = ( sa ** 2 ) / sb;
    #alpha = ( sa - sb ) / sa;             #f
    #ablandamiento = 1 / alpha;   # 1/f
    lat = la * ( math.pi / 180 );
    lon = lo * ( math.pi / 180 );
    Huso = np.fix( ( lo / 6 ) + 31);
    S = ( ( Huso * 6 ) - 183 );
    deltaS = lon - ( S * ( math.pi / 180 ) );
    Letra = ''
    if (la<-72):
        Letra='C';
    elif (la<-64):
        Letra='D';
    elif (la<-56):
        Letra='E';
    elif (la<-48):
        Letra='F';
    elif (la<-40):
        Letra='G';
    elif (la<-32):
        Letra='H';
    elif (la<-24):
        Letra='J';
    elif (la<-16):
        Letra='K';
    elif (la<-8):
        Letra='L';
    elif (la<0):
        Letra='M';
    elif (la<8):
        Letra='N';
    elif (la<16):
        Letra='P';
    elif (la<24):
        Letra='Q';
    elif (la<32):
        Letra='R';
    elif (la<40):
        Letra='S';
    elif (la<48):
        Letra='T';
    elif (la<56):
        Letra='U';
    elif (la<64):
        Letra='V';
    elif (la<72):
        Letra='W';
    else:
        Letra='X';

    a = math.cos(lat) * math.sin(deltaS);
    epsilon = 0.5 * math.log( ( 1 +  a) / ( 1 - a ) );
    nu = math.atan( math.tan(lat) / math.cos(deltaS) ) - lat;
    v = ( c / ( ( 1 + ( e2cuadrada * ( math.cos(lat) ) ** 2 ) ) ) ** 0.5 ) * 0.9996;
    ta = ( e2cuadrada / 2 ) * epsilon ** 2 * ( math.cos(lat) ) ** 2;
    a1 = math.sin( 2 * lat );
    a2 = a1 * ( math.cos(lat) ) ** 2;
    j2 = lat + ( a1 / 2 );
    j4 = ( ( 3 * j2 ) + a2 ) / 4;
    j6 = ( ( 5 * j4 ) + ( a2 * ( math.cos(lat) ) ** 2) ) / 3;
    alfa = ( 3 / 4 ) * e2cuadrada;
    beta = ( 5 / 3 ) * alfa ** 2;
    gama = ( 35 / 27 ) * alfa ** 3;
    Bm = 0.9996 * c * ( lat - alfa * j2 + beta * j4 - gama * j6 );
    xx = epsilon * v * ( 1 + ( ta / 3 ) ) + 500000;
    yy = nu * v * ( 1 + ta ) + Bm;
    if yy<0:
        yy=9999999+yy;
    x=xx;
    y=yy;
    utmzone = "%02d %c" % (Huso,Letra)
    return x,y,utmzone

def mySmoothPoints(la,lo,spacing,utmz):
    wla = []; wlo = []; u=[];
    for i in range(len(la)-1):
        w1 = np.array([[la[i+1]],[lo[i+1]]])
        wi = np.array([[la[i]],[lo[i]]])
        v = w1-wi;
        d = math.sqrt((la[i+1]-la[i])**2+(lo[i+1]-lo[i])**2);
        num_points_that_fit = math.ceil(d/spacing);
        vd = (v/np.linalg.norm(v)) * spacing;
        for k in range(num_points_that_fit):
            wla.append( (wi[0]+vd[0]*k) );
            wlo.append( (wi[1]+vd[1]*k) );
            u.append(utmz);

    wla.append( (la[len(la)-1]) )
    wlo.append( (lo[len(lo)-1]) )
    u.append(utmz);
    return wla,wlo,u

xx=[]
yy=[]
uu=[]

# convert all coarse points to utm coordinates
for i in range(len(thelatt)):
    [txx,tyy,tuu] = deg2utm(thelatt[i],thelonn[i])
    xx.append(txx)
    yy.append(tyy)
    uu.append(tuu)

# smooth coarse waypoints
[sxx,syy,suu] = mySmoothPoints(xx,yy,float(sys.argv[1]),uu[0])

f = open('C:\\Users\\gtg498u\\Documents\\MATLAB\\trover\\pythelat.txt', 'w')
for i in sxx:
    f.write(str(i).strip('[]') + "\n");

f = open('C:\\Users\\gtg498u\\Documents\\MATLAB\\trover\\pythelonn.txt', 'w')
for i in syy:
    f.write(str(i).strip('[]') + "\n");

f = open('C:\\Users\\gtg498u\\Documents\\MATLAB\\trover\\pytheutmzone.txt', 'w')
for i in suu:
    f.write(str(i).strip('[]') + "\n");

#print(str(sxx[i]))
print(len(sxx))
