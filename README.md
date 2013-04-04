# gopro_calibration: Calibrate a camera given an MP4 file with a calibration dance

## Usage

### To calibrate a camera
- Record the calibration dance and save it to videos/calibration_dance.mp4
- Put a default camera_calibration.txt file in your home directory, and then run calibrate
```bash
cp calibrations/gopro3_parameters.txt ~
roslaunch gopro_calibration calibrate.launch
```

- Click 'Calibrate', when all the bars go green
- Play another video using gscam, if the calibration is not done by the time the video is done
```bash
roslaunch gopro_calibration play.launch
```

- Click 'Save'

### To convert saved calibration to a yaml
```bash
tar xf /tmp/calibrationdata.tar.gz ost.txt
cp ost.txt ost.ini
rosrun camera_calibration_parsers convert ost.ini camera.yaml
mv ost.txt ~/camera_calibration.txt
```

### Play video and save rectified version
```bash
mkdir /tmp/images
roslaunch gopro_calibration play_and_save.launch
roscd gopro_calibration
mencoder "mf://*.jpg" -mf type=jpg:fps=15 -o output.mpg -speed 1 -ofps 30 -ovc lavc -lavcopts vcodec=mpeg2video:vbitrate=2500 -oac copy -of mpeg
```
  

