from pyax12.connection import Connection
from pyax12.argparse_default import common_argument_parser
import numpy as np
import time
import pygame
from scipy.signal import correlate
import RTIMU
import sys, getopt

sys.path.append('.')
import os.path



def calcularFase(y1,y2,dt,w):

    xcorr=correlate(y1,y2)
    nsamples=np.size(y1)
    tiempo = np.arange(1-nsamples, nsamples)
    return dt*tiempo[xcorr.argmax()]*w

def main():

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

    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)
    #poll_interval = imu.IMUGetPollInterval()
    pygame.init()
    pygame.joystick.init()

    stick = pygame.joystick.Joystick(0)
    stick.init()

    # Parse options
    #parser = common_argument_parser(desc=main.__doc__)
    #args = parser.parse_args()

    # Connect to the serial port
    #sc = Connection(port=args.port,
                                   #baudrate=args.baudrate,
                                   #timeout=args.timeout,
                                   #rpi_gpio=args.rpi)

    sc = Connection(port="/dev/ttyUSB0", baudrate=1000000)    
    dynamixel_id = 1
    
    freq=0.9
    w=2*np.pi*freq
    Am=45

    iniciar=False
    while(not iniciar):
        pygame.event.get()
        iniciar=stick.get_button(1)

    sc.set_cw_angle_limit(dynamixel_id,-150,degrees=True)
    sc.set_ccw_angle_limit(dynamixel_id,150,degrees=True) 

    sc.goto(dynamixel_id, 0, speed=512, degrees=True)
    time.sleep(1)
    start=time.time()
    offseto=0
    tant=time.time()-start
    pressed=False
    Ndatos=120
    datosroll=np.zeros([int(Ndatos/2)])
    datospmotor=np.zeros([int(Ndatos/2)])

    #contador para la ventana de tiempo
    i=0

    dt=0.01

    fase=np.pi/2
    K=0.5
    K2=0.5
    #Amplitud deseada del roll
    Ades=15
    Amax=Ades
    while(not pressed):
        
        A=Am+int(K2*(Ades-Amax))


        error=np.pi/2-fase
        pygame.event.get()
        pressed=bool(stick.get_button(2))
        tiempo=time.time()-start

        arg=tiempo*w
        offsetn=round(stick.get_axis(0),2)
        der=A*(offsetn-offseto)/(tiempo-tant)
        vel=int(np.abs((60*A*w*np.cos(w*tiempo+K*error)-60*der)*1023/(114*360)))

        if vel > 1023:
            vel=1023

        while(arg > 2*np.pi):
            arg-=2*np.pi

        if arg <= np.pi/2 or (arg>=3*np.pi/2):
            objetivo=A-int(A*offsetn)
            sc.goto(dynamixel_id, objetivo, speed=vel, degrees=True)

        else:
            objetivo=-A-int(A*offsetn)
            sc.goto(dynamixel_id, objetivo, speed=vel, degrees=True)

        #print(objetivo," ",vel," ",int(60*der*1023/(114*360)))clu
        pmotor=sc.get_present_position(dynamixel_id,degrees=True)

        #Daots de la IMU
        leyo=False
        if imu.IMURead():
        # x, y, z = imu.getFusionData()
        # print("%f %f %f" % (x,y,z))
            data = imu.getIMUData()
            fusionPose = data["fusionPose"]
            roll=np.degrees(fusionPose[1])
         
            leyo=True
        
        tant=time.time()-start
        offseto=offsetn

        if i%2==0 and leyo:
            datosroll[int(i/2)]=roll
            datospmotor[int(i/2)]=pmotor

        if i==Ndatos-1:
            fase=calcularFase(datospmotor,datosroll,dt,w)
            Amax=int(np.amax(datosroll))
            #print(Amax,fase)

        i+=1
        if i >= Ndatos:
            i=0

        time.sleep(dt)
        #time.sleep(poll_interval*1.0/1000.0)

    # Close the serial connection
    sc.close()

    while(True):
        pygame.event.get()
        x=bool(stick.get_button(1))
        if x:
            main()
        triangulo=bool(stick.get_button(3))
        if triangulo:
            break
if __name__ == '__main__':
    main()
