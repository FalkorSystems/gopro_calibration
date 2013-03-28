#!/usr/bin/python
import sys
import cv2
import yaml
import numpy

def progress_text(status):
    global last_status
    try:
        last_status
    except:
        last_status = 0.0
        sys.stdout.write("Processing: [                    ]\rProcessing: [")
        sys.stdout.flush()
    if (status == 1.0):
        sys.stdout.write('.')
        print ""
    elif (status-last_status >= 0.05):
        sys.stdout.write('.')
        sys.stdout.flush()
        last_status = last_status + 0.05

def cvmat_constructor(loader, node):
    data = loader.construct_mapping(node,deep=True)
    matrix = cv2.cv.CreateMat(data['rows'], data['cols'], cv2.CV_32FC1)
    for r in range(data['rows']):
        for c in range(data['cols']):
            cv2.cv.SetReal2D(matrix, r, c, data['data'][c+r*data['cols']])
    return numpy.asarray(matrix)

def load_calibration(filename):
    yaml.add_constructor('tag:yaml.org,2002:opencv-matrix', cvmat_constructor)
    try:
        return yaml.load(file(filename, 'r'))
    except yaml.YAMLError, e:
        print "ERROR: configuration file:", e

def defisheye(input_filename,output_filename,calibration_filename,update=None):
    cfg = load_calibration(calibration_filename)
    cameraMatrix = cfg["camera_matrix"]
    distCoeffs = cfg["distortion_coefficients"]
    imageSize = (cfg["image_width"],cfg["image_height"])

    video_in = cv2.VideoCapture(input_filename)
    video_frames = int(video_in.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT))
    video_fps = video_in.get(cv2.cv.CV_CAP_PROP_FPS)
    video_width = int(video_in.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH))
    video_height = int(video_in.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT))

    print "Frames: " + str(video_frames)
    print "FPS: " + str(video_fps)
    print "Width: " + str(video_width)
    print "Height: " + str(video_height)

    video_out = cv2.VideoWriter(
        filename=output_filename,
        fourcc=cv2.cv.CV_FOURCC('T','H','E','O'),
        fps=video_fps,
        frameSize=(video_width,video_height),
        isColor=1)

    #(map1, map2) = cv2.initUndistortRectifyMap(cameraMatrix, distCoeffs, None, cv2.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0)[0], imageSize, cv2.CV_16SC2);
    (map1, map2) = cv2.initUndistortRectifyMap(numpy.asarray(cameraMatrix), numpy.asarray(distCoeffs), None, numpy.asarray(cameraMatrix), imageSize, cv2.CV_16SC2);

    for f in range(video_frames):
        if (update != None):
            pos = video_in.get(cv2.cv.CV_CAP_PROP_POS_AVI_RATIO)
            update(pos)

        image_in = video_in.read()[1]
        if image_in is None:
            print "Frame ", f, " end."
            break
        image_rect = cv2.remap(image_in, map1, map2, cv2.INTER_CUBIC)
        video_out.write(image_rect)

defisheye(sys.argv[1],"defisheye.ogv","camera.yml",progress_text)
