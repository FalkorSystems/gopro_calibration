# gopro_calibration: A ROS package to calibrate a GoPro camera

## Usage

- To calibrate a camera
-- Record the calibration dance and save it to videos/calibration_dance.mp4
-- Put a default camera_calibration.txt file in your home directory
   ```cp calibrations/gopro3_parameters.txt ~```
-- Calibrate
   ```roslaunch gopro_calibration calibrate.launch```
-- Click 'Calibrate'
-- Play another video using gscam
   ```roslaunch gopro_calibration play.launch```
-- Click 'Save'

- To convert saved calibration to a yaml
-- Extract ost.txt file
   ```tar xf /tmp/calibrationdata.tar.gz ost.txt```
-- Rename
   ```mv ost.txt ost.ini```
-- Convert to YAML
   ```rosrun camera_calibration_parsers convert ost.ini camera.yaml```

- Place ost.txt file in home directory
-- Extract ost.txt file
   ```tar xf /tmp/calibrationdata.tar.gz ost.txt```
-- Rename
   ```mv ost.txt ~/camera_calibration.txt```

- Play video and save rectified version
  ```mkdir /tmp/images```
  ```roslaunch gopro_calibration play_and_save.launch```
  ```roscd gopro_calibration```
  ```mencoder "mf://*.jpg" -mf type=jpg:fps=15 -o output.mpg -speed 1 -ofps 30 -ovc lavc -lavcopts vcodec=mpeg2video:vbitrate=2500 -oac copy -of mpeg```

  

