#!/usr/bin/python
"""
  First test with ardrone "Isabel"
  usage:
       ./ver0.py fly
"""
import pygame
import datetime

import sys
import os

# martin libraries
sys.path.append( ".."+os.sep+"heidi") 
from ardrone2 import ARDrone2, ManualControlException, manualControl
from sourcelogger import SourceLogger


def pygameTest():
    pygame.init()
    screen = pygame.display.set_mode((100,100))
    while 1:
        # for event in pygame.event.get():
            # if event.type == pygame.QUIT: sys.exit()
        events = pygame.event.get()
        print events
        for event in events:
	    print event
            if event.type == pygame.KEYDOWN:
		return
                if event.key == pygame.K_LEFT:
                    return


def linuxKbHit():
        events = pygame.event.get()
        for event in events:
	
	    if event.type == pygame.KEYDOWN:
		return True
	return False
     

def upAndDown(replayLog, metaLog):
  if replayLog == None:
      pygame.init()
      screen = pygame.display.set_mode((100,100))
  #drone = ARDrone2(replayLog = "logs/navdata_140128_175017.log.gz")
  drone = ARDrone2(replayLog, metaLog = metaLog, console = linuxKbHit, speed = 0.1)
  drone.startVideo()
  try:
      drone.wait(1.0)
      drone.takeoff()
      drone.setVideoChannel( front=False )
      drone.hover(1.0)
      drone.turnRight(10.0)
      drone.setVideoChannel( front=True )
      drone.hover(1.0)
      #for i in xrange(10):
       #   print drone.time, drone.acc, drone.coord
        #  drone.update()
      drone.land()
      drone.wait(1.0)
  except ManualControlException, e:
      print "ManualControlException"
      drone.land()
  drone.wait(1.0)
  drone.stopVideo()

if __name__ == "__main__":
  if len(sys.argv) < 2:
    print __doc__ # it prints first comment
    sys.exit(2)
  print sys.argv
  if len(sys.argv) > 2:
      metaLog = open(sys.argv[2])
      for line in metaLog:
          if line.startswith("navdata:"):
              replayLog = line.split()[1].strip()
              break
  else:
      metaLog = open( datetime.datetime.now().strftime("logs/meta_%y%m%d_%H%M%S.log"), "w" )
      metaLog.write( str(sys.argv) + "\n" )    
      metaLog.flush()
      replayLog = None
  upAndDown(replayLog, metaLog)
  #pygameTest()
