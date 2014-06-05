#!/usr/bin/python
"""
  Field Robot Event, task 4
  usage:
       ./fre_task4.py <TODO>
"""
# based on airrace_drone.py
import sys
import os
import datetime
import multiprocessing
import cv2
import math
import numpy as np

# martin libraries
sys.path.append( ".."+os.sep+"heidi") 

from pave import PaVE, isIFrame, frameNumber, timestamp, correctTimePeriod, frameEncodedWidth, frameEncodedHeight

from sourcelogger import SourceLogger
from ardrone2 import ARDrone2, ManualControlException, manualControl, normalizeAnglePIPI, distance
import viewlog
from line import Line
from pose import Pose

from airrace import main as imgmain # image debugging TODO move to launcher
from airrace import saveIndexedImage, FRAMES_PER_INDEX

import cvideo
import cimg


MAX_ALLOWED_SPEED = 0.8
MAX_ALLOWED_VIDEO_DELAY = 2.0 # in seconds, then it will wait (desiredSpeed = 0.0)


def timeName( prefix, ext ):
    dt = datetime.datetime.now()
    filename = prefix + dt.strftime("%y%m%d_%H%M%S.") + ext
    return filename

def getOrNone():
    if g_queueResults.empty():
        return None
    return g_queueResults.get()


g_pave = None
g_processingEnabled = True # shared multiprocess variable (!)

g_img = None


def wrapper( packet ):
    global g_pave
    global g_img
    if g_pave == None:
        g_pave = PaVE()
        cvideo.init()
        g_img = np.zeros([720,1280,3], dtype=np.uint8)
    g_pave.append( packet )
    header,payload = g_pave.extract()
    while payload:
        if isIFrame( header ):
            w,h = frameEncodedWidth(header), frameEncodedHeight(header)
            if g_img.shape[0] != h or g_img.shape[1] != w:
                print g_img.shape, (w,h)
                g_img = np.zeros([h,w,3], dtype=np.uint8)
            ret = cvideo.frame( g_img, isIFrame(header) and 1 or 0, payload )
            frame = g_img
            assert ret
            if ret:
                result = None
                if g_processingEnabled:
                    result = processFrame( frame, debug=False )
                return (frameNumber( header ), timestamp(header)), result
        header,payload = g_pave.extract()


def downloadOldVideo( drone, timeout ):
    "download video if it is delayed more than MAX_VIDEO_DELAY"
    global g_processingEnabled
    restoreProcessing = g_processingEnabled
    print "Waiting for video to start ..."
    startTime = drone.time
    while drone.time < startTime + timeout:
        if drone.lastImageResult != None:
            (frameNumber, timestamp), rects = drone.lastImageResult
            videoTime = correctTimePeriod( timestamp/1000., ref=drone.time )
            videoDelay = drone.time - videoTime
            print videoDelay
            if videoDelay < MAX_ALLOWED_VIDEO_DELAY:
                print "done"
                break
            g_processingEnabled = False
        drone.update()
    g_processingEnabled = restoreProcessing


g_queueResults = multiprocessing.Queue()

class FreTask4Drone( ARDrone2 ):
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


def processFrame( frame, debug=False ):
    cimg.green( frame, 1.1 )
    cv2.imshow('image', frame)
    saveIndexedImage( frame )

def freTask4(drone):
    drone.speed = 0.1
    maxVideoDelay = 0.0
    maxControlGap = 0.0
    desiredSpeed = MAX_ALLOWED_SPEED

    try:
        drone.wait(1.0)
        drone.setVideoChannel( front=False )
        downloadOldVideo( drone, timeout=20.0 )
        drone.takeoff()
        drone.hover(20.0)

    except ManualControlException, e:
        print "ManualControlException"
        if drone.ctrlState == 3: # CTRL_FLYING=3 ... i.e. stop the current motion
            drone.hover(0.1)
    drone.land()
    drone.wait(1.0)
    drone.stopVideo()
    print "MaxVideoDelay", maxVideoDelay
    print "MaxControlGap", maxControlGap
    print "Battery", drone.battery


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "img":
        imgmain( sys.argv[1:], processFrame )
        sys.exit( 0 )
    import launcher
    launcher.launch( sys.argv, FreTask4Drone, freTask4 )
     
