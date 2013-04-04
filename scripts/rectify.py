#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, I Heart Engineering, Falkor Systems, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
# * Neither the name of I Heart Engineering nor Falkor Systems nor the
#   names of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Original from
# http://www.htw-mechlab.de/index.php/undistortion-der-gopro-hd-hero2/

import sys
import os
import cv
import numpy as np
from progressbar import *

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
    outfilename = filebase + "_rect.avi"
    print "writing to " + outfilename

    writer = cv.CreateVideoWriter(
        filename= outfilename,
        fourcc=cv.FOURCC('F', 'M', 'P', '4'),
        fps=fps,
        frame_size=(width,height),
        is_color=1)
 
    map1 = cv.CreateImage((width, height), cv.IPL_DEPTH_32F, 1)
    map2 = cv.CreateImage((width, height), cv.IPL_DEPTH_32F, 1)
    cv.InitUndistortMap(camera_matrix, dist_coeffs, map1, map2)
 
    widgets = ['Rectifying: ', Percentage(), ' ', Bar(marker=RotatingMarker()),
               ' ', ETA() ]
    pbar = ProgressBar(widgets=widgets, maxval=nFrames).start()

    for f in xrange( nFrames ):
        pbar.update(f)
        frameImg = cv.QueryFrame( vidFile )
        if frameImg is None:
            print "Frame Nr", f, " fehlerhaft. Abbruch"
            break
        undistimage = cv.CloneImage(frameImg)
        cv.Remap(frameImg, undistimage, map1, map2)
 
        cv.WriteFrame(writer, undistimage)
          
        k = cv.WaitKey(1)
        if k % 0x100 == 27:
            # user has press the ESC key, so exit
            break

    pbar.finish()
    del writer

