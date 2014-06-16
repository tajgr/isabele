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

g_mser = None

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
#    rectangles = []
#    cimg.green( frame, 0.9 )
#    global g_mser
#    if g_mser == None:
#        g_mser = cv2.MSER( _delta = 10, _min_area=100, _max_area=360*250 )
#    b, g, r, = cv2.split( frame )
#    gray = g
#    contours = g_mser.detect(gray, None)
#    print len( contours )
#    for cnt in contours:
#        if len(cnt) > 10000:
#            print len(cnt)
#            rect = cv2.minAreaRect(cnt)
#            print rect
##        sys.exit()
#            rectangles.append(rect)
#    print "len(rectangles)"+str( len( rectangles ) )
#    if debug:
##        cv2.polylines(frame, contours, 2, (0, 255, 0), 2)
#        for rect in rectangles:
#            box = cv2.cv.BoxPoints(rect)
#            box = np.int0(box)
#            cv2.drawContours( frame,[box],0,(0,0,255),2)
#        cv2.imshow('image', frame)
#        saveIndexedImage( frame )
    refLine = [0, 0, 1, 1]
    return refLine


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
        startTime = drone.time
            while drone.time < startTime + 1.0:
                drone.update("AT*PCMD=%i,0,0,0,0,0\r") # drone.hover(1.0)
                # TODO sonar/vision altitude
                poseHistory.append( (drone.time, (drone.coord[0], drone.coord[1], drone.heading), (drone.angleFB, drone.angleLR), None) )
        magnetoOnStart = drone.magneto[:3]
        print "NAVI-ON"
        startTime = drone.time
        sx,sy,sz,sa = 0,0,0,0
        lastUpdate = None
        while drone.time < startTime + 600.0:
            altitude = desiredHeight
            if drone.altitudeData != None:
                altVision = drone.altitudeData[0]/1000.0
                altSonar = drone.altitudeData[3]/1000.0
                altitude = (altSonar+altVision)/2.0
                if abs(altSonar-altVision) > 0.5:
                    print altSonar, altVision
                    altitude = max( altSonar, altVision ) # sonar is 0.0 sometimes (no ECHO)

            sz = max( -0.2, min( 0.2, desiredHeight - altitude ))
            if altitude > 2.5:
                # wind and "out of control"
                sz = max( -0.5, min( 0.5, desiredHeight - altitude ))
            sx = max( 0, min( drone.speed, desiredSpeed - drone.vx ))

            if drone.lastImageResult:
                lastUpdate = drone.time
                assert len( drone.lastImageResult ) == 2 and len( drone.lastImageResult[0] ) == 2, drone.lastImageResult
                (frameNumber, timestamp), rects = drone.lastImageResult
                viewlog.dumpVideoFrame( frameNumber, timestamp )
                #TODO
                # keep history small
                videoTime = correctTimePeriod( timestamp/1000., ref=drone.time )
                videoDelay = drone.time - videoTime
                if videoDelay > 1.0:
                    print "!DANGER! - video delay", videoDelay
                maxVideoDelay = max( videoDelay, maxVideoDelay )
                toDel = 0
                for oldTime, oldPose, oldAngles, oldAltitude in poseHistory:
                    toDel += 1
                    if oldTime >= videoTime:
                        break
                poseHistory = poseHistory[:toDel]
                
                print "FRAME", frameNumber/15
#                TODO

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
     
