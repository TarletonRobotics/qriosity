#
# -*- coding: utf-8 -*-
"""firmata_test_3_18_19.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/15TGmCmLTn5gkujUXdemXobQZoy_62WtO
"""

import sys
import warnings
warnings.filterwarnings('ignore')
import math as math

from pyfirmata import ArduinoMega, util
import inputs
import time
import pygame


class Rover(object):
    """Create a rover object based on a pin map
    """
    def __init__(self, port, forward, reverse):
        """Return new rover object.
        analog pins will be pin numbers w/o A
        """
        self.port = port
        self.f = forward
        self.r = reverse
        
        
    def set_port_pins(self):
        board = ArduinoMega(self.port)
        """
        the firmata pin methods take a string
        """
        forward = 'd:'+str(labrat.f)+':p'
        reverse = 'd:'+str(labrat.r)+':p'
        self.pinForward = board.get_pin(forward)
        self.pinReverse = board.get_pin(reverse)
        self.pause = board.pass_time(5)

labrat=Rover('/dev/cu.usbmodem1411',3,5)

labrat.set_port_pins()

def joystick(rover):
    pygame.init()
    pygame.joystick.init()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
            # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN
            # JOYBUTTONUP JOYHATMOTION
            if event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.")
            if event.type == pygame.JOYBUTTONUP:
                print("Joystick button released.")
        
        # events = inputs.get_gamepad()
        # for event in events:
        #     print ('event.ev_type',event.ev_type, 
        #         'event.code',event.code, 'event.state',event.state)
        #     """
        #     directional pad up is forward, let off direction pad is stop
           
        #     """
        #     if event.code == 'ABS_Y' and event.state == 0:
        #         rover.pinForward.write(.2)
        #     if event.code == 'ABS_Y' and event.state ==127:
        #         rover.pinForward.write(0)
        #     if event.code == 'ABS_Y' and event.state == 255:
        #         #time.sleep(5)
        #         #rover.pause
        #         rover.pinForward.write(.2)
        #     if event.code == 'ABS_Y' and event.state ==127:
        #         rover.pinForward.write(0)

joystick(labrat)

