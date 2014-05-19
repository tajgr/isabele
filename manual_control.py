#!/usr/bin/python
"""
  First test with ardrone "Isabel"
  usage:
       ./ver0.py fly
"""

import datetime
import sys
import os
import multiprocessing
import cv2
import math
import numpy as np

# martin libraries
sys.path.append( ".."+os.sep+"heidi") 


from sourcelogger import SourceLogger
from ardrone2 import ARDrone2, ManualControlException, manualControl, normalizeAnglePIPI, distance



class ManualControlLinux( ARDrone2 ):
    def __init__( self, replayLog=None, speed = 0.2, skipConfigure=False, metaLog=None, console=None ):
        self.loggedVideoResult = None
        self.lastImageResult = None
        self.videoHighResolution = True
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

def manualControlPygame( drone ):
    speed = 0.2
    speedUpDown = 0.2
    while 1:
        events = pygame.event.get()
#        print events
        for event in events:
	    print event
            if event.type == pygame.KEYDOWN:
		if event.key == pygame.K_LEFT:
		    print "K_LEFT"
		    drone.moveXYZA( 0, 0, 0, -speed )		
		elif event.key == pygame.K_RIGHT:
		    print "K_RIGHT"
		    drone.moveXYZA( 0, 0, 0, speed )
		
		elif event.key == pygame.K_DOWN:
		    print "K_DOWN"
		    drone.moveXYZA( -speed, 0, 0, 0 )
		
		elif event.key == pygame.K_UP:
		    print "K_UP"
		    drone.moveXYZA( speed, 0, 0, 0 )
		
		elif event.key == pygame.K_PAGEUP:
		    print "K_PAGEUP"
		    drone.moveXYZA( 0, 0, speedUpDown, 0 )
		
		elif event.key == pygame.K_PAGEDOWN:
		    print "K_PAGEDOWN"
		    drone.moveXYZA( 0, 0, -speedUpDown, 0 )
		
                elif event.key == pygame.K_SPACE:
		    print "K_SPACE"
                    drone.hover( 0.5 )
                    
                elif event.key == pygame.K_RETURN:
		    print "K_RETURN"
		    return
                
                else:
		    return
	    else:
		drone.update()



def flyIsabelle( drone ):
    try:
        drone.wait(1.0)
        drone.takeoff()
        drone.hover(1.0)
        drone.setVideoChannel( front=False )
    except ManualControlException, e:
        print "ManualControlException"
        manualControlPygame( drone )
        drone.hover(0.5)
        drone.land()
    drone.wait(1.0)
    drone.stopVideo()

if __name__ == "__main__":
    import launcher
    launcher.launch( sys.argv, ManualControlLinux, flyIsabelle )
    