import sys, getopt

sys.path.append('.')
import RTIMU
import os.path
from threading import Thread
from pyax12.connection import Connection
import time
import math
from scipy.signal import find_peaks
import numpy as np

file=open("results.txt","w")
file.write("Time [s];Frequency [Hz];Target Pos[degrees];Present Pos[degrees];Moving speed [Degress/s];Roll [Degrees];Pitch [Degrees];Yaw [Degrees];ax[g];ay[g];az[g]\n")
SETTINGS_FILE = "RTIMULib"

print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
  print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)

print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):
    print("IMU Init Failed")
    sys.exit(1)
else:
    print("IMU Init Succeeded")

# this is a good time to set any fusion parameters

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)

sc = Connection(port="/dev/ttyAMC0", baudrate=1000000)
id = 1
f = 0
Am=50
sp=50
sc.goto(id, Am, speed=256, degrees=True)
time.sleep(1)
target=0
vel=0
error=0
beta=0
gamma=0
roll=0
Pmreal=0
Vmreal=0
phi=0
class ReadFreq(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.start()
    def run(self):
        global vel
        global f
        while True:
            vel=eval(input("Frequency: "))
            if vel!=f:
                f=vel
class Comparator(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.start()
    def run(self):   
        global roll
        global Aroll
        global Amreal
        global Pmreal
        global beta
        global gamma
        global error
        global Am
        global rollr
        global phase
        global b
        global g
        global rolllist
        phase=math.pi*0.5
        L=400
        rolllist=np.full(L,1)
        Pmreallist=np.full(L,1)
        error=0
        rollr=0
        while True:
            offset=np.mean(rolllist)
            for j in range(0,L-2):
                rollp=rolllist[j+1]-offset
                Pmrealp=Pmreallist[j+1]
                rolllist[j]=rollp
                Pmreallist[j]=Pmrealp
            
            rolllist[L-1]=(roll+rolllist[L-2])/2-offset
            rollr=rolllist[L-1]       
            
            Pmreallist[L-1]=Pmreal
            peaksroll, _ =find_peaks(rolllist, prominence=0)
                        
            Aroll=np.absolute(np.nanmean(rolllist[peaksroll]))
            Amreal=np.absolute(np.amax(Pmreallist))
            Aroll=max(Aroll,0.1)
            b=Pmreal/Amreal
            g=rollr/Aroll
            b=max(min(b,1),-1)
            g=max(min(g,1),-1)

            beta=math.asin(b)
            gamma=math.asin(g)
            error=phase-(gamma-beta)
            time.sleep(0.01)
            
class MotorAct(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.start()
    def run(self):
        global angle
        global x
        global target
        global f
        global Pmreal
        global Vmreal
        global rollr
        t1=0
        phi=0
        K=0
        while True:
            inicio=time.time()
            phi=error*K
            #angle=x*10
            #target=int(50*math.cos(hz*math.radians(angle)))
            target=int(Am*math.sin(f*2*math.pi*t1-phi))
            #print(target)
            #sp=math.floor(math.fabs(620*math.sin(hz*math.radians(angle))))
            sp=math.floor(math.fabs(2*1.5*Am*math.pi*f*math.cos(math.acos(target/Am))))
            sp=int(max(min(sp, 620), 2))
            #print(sp)
            Pmreal=sc.get_present_position(id,degrees=True)
            Vmreal=sc.get_present_speed(id)*0.67
            #load=sc.get_present_load(id)
            #volt=sc.get_present_voltage(id)
            sc.goto(id, target, speed=sp, degrees=True)
            #x+=1
            fin=time.time()
            t1=t1+(fin-inicio)

class DataAcq(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.start()
    def run(self):
        global target
        global f
        global t0
        global roll
        global Pmreal
        global Vmreal
        t0=0
        
        while True:
            if (imu.IMURead() and f>0):
                # x, y, z = imu.getFusionData()
                # print("%f %f %f" % (x,y,z))
                data = imu.getIMUData()
                
                fusionPose = data["fusionPose"]
                roll=math.degrees(fusionPose[0])
                pitch=math.degrees(fusionPose[1])
                yaw=math.degrees(fusionPose[2])
                
                accelVals = data["accel"]
                ax=accelVals[0]
                ay=accelVals[1]
                az=accelVals[2]
                
                file.write("%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f\n" % (t0,f,target,Pmreal,Vmreal,roll,pitch,yaw,ax,ay,az))                
                time.sleep(0.01)
                t0=t0+0.01
                
                #print(g)
            

ReadFreq()
Comparator()
MotorAct()
DataAcq()
while True:
    pass
