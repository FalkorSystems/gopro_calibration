#!/usr/bin/env python
# Original from
# http://www.htw-mechlab.de/index.php/undistortion-der-gopro-hd-hero2/

import sys
import os
import cv
import numpy as np
 
if __name__ == '__main__':
    filename = sys.argv[1]

    vidFile = cv.CaptureFromFile( filename )
    nFrames = int(  cv.GetCaptureProperty( vidFile, cv.CV_CAP_PROP_FRAME_COUNT ) )
    fps = cv.GetCaptureProperty( vidFile, cv.CV_CAP_PROP_FPS )
    waitPerFrameInMillisec = int( 1/fps * 1000/1 )
    width = int( cv.GetCaptureProperty(vidFile, cv.CV_CAP_PROP_FRAME_WIDTH))
    height = int( cv.GetCaptureProperty(vidFile, cv.CV_CAP_PROP_FRAME_HEIGHT))
 
    print 'Num. Frames = ', nFrames
    print 'Frame Rate = ', fps, ' frames per sec'
 
    # Camera Daten fuer GoPro HD Hero2
    camera_matrix = cv.CreateMat(3, 3, cv.CV_32FC1)
    cv.SetReal2D(camera_matrix, 0, 0, 580.04414)
    cv.SetReal2D(camera_matrix, 0, 1, 0.0)
    cv.SetReal2D(camera_matrix, 0, 2, 678.13347)
    cv.SetReal2D(camera_matrix, 1, 0, 0.0)
    cv.SetReal2D(camera_matrix, 1, 1, 603.16324)
    cv.SetReal2D(camera_matrix, 1, 2, 362.61503)
    cv.SetReal2D(camera_matrix, 2, 0, 0.0)
    cv.SetReal2D(camera_matrix, 2, 1, 0.0)
    cv.SetReal2D(camera_matrix, 2, 2, 1.0)
 
    dist_coeffs = cv.CreateMat(1, 5, cv.CV_32FC1)
    cv.SetReal2D(dist_coeffs, 0, 0, -0.20605)
    cv.SetReal2D(dist_coeffs, 0, 1, 0.03435)
    cv.SetReal2D(dist_coeffs, 0, 2, 0.00055)
    cv.SetReal2D(dist_coeffs, 0, 3, -0.00115)
    cv.SetReal2D(dist_coeffs, 0, 4, 0.00000)
 
    # neuen Film vorbereiten
    filebase, fileext = os.path.splitext(filename)
#    outfilename = filebase + "_rect.avi"
#    print "writing to " + outfilename

#    writer = cv.CreateVideoWriter(
#        filename= outfilename,
#        fourcc=cv.CV_FOURCC('M', 'P', '1', 'V'),
#        fps=fps,
#        frame_size=(width,height),
#        is_color=1)
 
    map1 = cv.CreateImage((width, height), cv.IPL_DEPTH_32F, 1)
    map2 = cv.CreateImage((width, height), cv.IPL_DEPTH_32F, 1)
    cv.InitUndistortMap(camera_matrix, dist_coeffs, map1, map2)
 
    for f in xrange( nFrames ):
        frameImg = cv.QueryFrame( vidFile )
        if frameImg is None:
            print "Frame Nr", f, " fehlerhaft. Abbruch"
            break
        undistimage = cv.CloneImage(frameImg)
        cv.Remap(frameImg, undistimage, map1, map2)
 
        cv.ShowImage( "Video",  undistimage )
        cv.SaveImage( "%s_%09d.jpg" % ( filebase, f ), undistimage )
#       cv.WriteFrame(writer, undistimage)
 
        #Progress Bar
#        prozent = f*100/nFrames
#        print prozent, "%"
         
        k = cv.WaitKey(1)
        if k % 0x100 == 27:
            # user has press the ESC key, so exit
            break
 
    cv.DestroyWindow( "Video" )
#   del writer

print "Now executing:"
fpsi = round(fps)
command = "mencoder \"mf://%s_*.jpg\" -mf type=jpg:fps=%d -o %s_rect.mpg -speed 1 -ofps %d -ovc lavc -lavcopts vcodec=mpeg2video:vbitrate=2500 -oac copy -of mpeg" % ( filebase, fpsi, filebase, fpsi )

print command
os.system(command)

command = "rm %s_*.jpg" % filebase
print command
os.system(command)
