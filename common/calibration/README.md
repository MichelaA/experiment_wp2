PHASE-SPACE/KINECT CALIBRATION
==============================

The procedures is to estimate the pose of the pattern in both systems, and then compute the relative transformation between them.
For now, it works only in the phase-space PC due to local files in there.
ToDo: make it portable to any computer.

Using the bracelet as a pattern:

1. Connect to "Internet" and then "Local\_Network\_PS" networks
2. Ensure that the .PCD and .PLY files of the bracelet are in the folder ~/poseEstimateion2/data in their respective subfolders
3. `roslaunch calibration kinect.launch`
4. (optional if you want to visualize the bracelet) `roslaunch glove display.launch`
5. (optional if you want to visualize the bracelet) `rqt`
6. Calibrate by running: `rosrun calibration kinect_calibration` In the screen, you will see a line like: `trans: x y z qw qx qy qz` that defines the calibrated pose between the phase-space (`world` frame) and the kinect (`camera_rgb_optical_frame` frame), so copy this values
in the package `rgbd_sensor/RGBDsensor.launch` 
7. (If you ran 3. and 4.) In order to check the result, launch the previous modified file
`roslaunch rgbd_sensor RGBDsensor.launch`
You should see in rviz the point cloud aligned with the reconstruction of the bracelet overlaid properly.






