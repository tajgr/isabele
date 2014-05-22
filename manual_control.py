#!/usr/bin/python
"""
    Manual control flying via pygame.
        usage:
            python manulal_control.py <TODO>
"""

import datetime
import sys
import os
import multiprocessing
import cv2
import math
import numpy as np
import pygame

# martin libraries
sys.path.append( ".."+os.sep+"heidi") 


from sourcelogger import SourceLogger
from ardrone2 import ARDrone2, ManualControlException, manualControl, normalizeAnglePIPI, distance
from rr_drone import timeName, getOrNone, wrapper


g_queueResults = multiprocessing.Queue()

class ManualControlLinux( ARDrone2 ):
    def __init__( self, replayLog=None, speed = 0.2, skipConfigure=False, metaLog=None, console=None ):
        self.loggedVideoResult = None
        self.lastImageResult = None
        self.videoHighResolution = False
        ARDrone2.__init__( self, replayLog, speed, skipConfigure, metaLog, console )
        if replayLog == None:
            name = timeName( "logs/src_cv2_", "log" ) 
            metaLog.write("cv2: "+name+'\n' )
            self.loggedVideoResult = SourceLogger( getOrNone, name ).get
            self.startVideo( wrapper, g_queueResults, record=True, highResolution=self.videoHighResolution )
        else:
            assert metaLog
            self.loggedVideoResult = SourceLogger( None, metaLog.getLog("cv2:") ).get
            self.startVideo( record=True, highResolution=self.videoHighResolution )

    def update( self, cmd="AT*COMWDG=%i,\r" ):
        ARDrone2.update( self, cmd )
        if self.loggedVideoResult != None:
            self.lastImageResult = self.loggedVideoResult()

def manualControlPygame( drone, desiredHeight = 2.0 ):
    speed = 0.1
    speedY = 0.1
    speedA = 0.4
    heightStep = 0.1
    sx, sy, sz, sa = 0, 0, 0, 0
    while 1:
        events = pygame.event.get()
#        print events
        for event in events:
            #print event
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    #print "K_LEFT"
                    sa = speedA
                    
                elif event.key == pygame.K_RIGHT:
                    #print "K_RIGHT"
                    sa = -speedA
                
		elif event.key == pygame.K_DOWN:
		    #print "K_DOWN"
                    sx = -speed
		
		elif event.key == pygame.K_UP:
		    #print "K_UP"
                    sx = speed
		
		elif event.key == pygame.K_u:
		    #print "K_PAGEUP"
                    desiredHeight = desiredHeight + heightStep
                    print "Current desired height ", desiredHeight
                    
		elif event.key == pygame.K_d:
		    #print "K_PAGEDOWN"
                    desiredHeight = desiredHeight - heightStep
                    print "Current desired height ", desiredHeight
                
                elif event.key == pygame.K_PAGEDOWN:
                    sy = speedY
                
                elif event.key == pygame.K_PAGEUP:
                    sy = -speedY
		
                elif event.key == pygame.K_SPACE:
		    #print "K_SPACE"
                    drone.hover( 0.5 )
                    
                elif event.key == pygame.K_RETURN:
		    #print "K_RETURN"
		    return
                
                else:
		    return
            if event.type == pygame.KEYUP:
                sx, sy, sz, sa = 0, 0, 0, 0
         
        altitude = desiredHeight
        if drone.altitudeData != None:
            altVision = drone.altitudeData[0]/1000.0
            altSonar = drone.altitudeData[3]/1000.0
            altitude = (altSonar+altVision)/2.0
            if abs(altSonar-altVision) > 0.5:
                print altSonar, altVision
                altitude = max( altSonar, altVision ) # sonar is 0.0 sometimes (no ECHO)

        sz = max( -0.2, min( 0.2, desiredHeight - altitude ))
        
        drone.moveXYZA( sx, sy, sz, sa )



def flyIsabelle( drone ):
    try:
        drone.wait(1.0)
        drone.setVideoChannel( front=False )
        drone.takeoff()
        drone.hover( 10.0)
    except ManualControlException, e:
        print "ManualControlException"
        manualControlPygame( drone )
    drone.hover(0.5)
    drone.land()
    drone.wait(1.0)
    drone.stopVideo()
    print "Battery", drone.battery

if __name__ == "__main__":
    import launcher
    launcher.launch( sys.argv, ManualControlLinux, flyIsabelle )
    