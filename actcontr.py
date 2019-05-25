#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# PyAX-12

# The MIT License
#
# Copyright (c) 2010,2015 Jeremie DECOCK (http://www.jdhp.org)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#from pyax12.connection import Connection
#from pyax12.argparse_default import common_argument_parser
import numpy as np
import time
import pygame



def main():

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

    #dynamixel_id = args.dynamixel_id
    
    freq=1.1
    w=2*np.pi*freq
    A=45

    iniciar=False
    while(not iniciar):
        pygame.event.get()
        iniciar=stick.get_button(1)

    #sc.goto(dynamixel_id, 0, speed=512, degrees=True)
    start=time.time()
    offseto=0
    tant=time.time()-start
    pressed=False
    while(not pressed):
        pressed=bool(stick.get_button(2))
        tiempo=time.time()-start
        pygame.event.get()
        arg=tiempo*w
        offsetn=round(stick.get_axis(0),2)
        der=A*(offsetn-offseto)/(tiempo-tant)
        vel=np.abs(int((60*A*w*np.cos(w*tiempo)+60*der)*1023/(114*360)))
        if vel > 1023:
            vel=1023
        while(arg > 2*np.pi):
            arg-=2*np.pi

        if arg <= np.pi/2 or (arg>=3*np.pi/2):
            objetivo=A+int(A*offsetn)
            #sc.goto(dynamixel_id, objetivo, speed=vel, degrees=True)
        else:
            objetivo=-A+int(A*offsetn)
            #sc.goto(dynamixel_id, objetivo, speed=vel, degrees=True)
        print(objetivo," ",vel," ",int(60*der*1023/(114*360)))
        tant=time.time()-start
        offseto=offsetn
        time.sleep(0.05)
    # Close the serial connection
    #serial_connection.close()   
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
