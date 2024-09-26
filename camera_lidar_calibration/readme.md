# Calibration for Camera and Lidar
### 1. collect rosbags
- The bags should contain the check board in different orientations and distances to the camera
- The margins between the boundary of the board and the checkerboard should be measured, as well as the grid sizes

### 2. process the rosbag
```commandline
python process_rosbags.py
```
prameters:
- ```--lidar_topic="" --camera_topic=""``` ros topics 
- ```--output_door=""``` output folder 
- ```--bag=""``` bag directory
- ```--only_camera``` if only process camera information
- ```--compress_image``` if the image is compressed type

### 3. Follow the links for calibration:
- Camera Calibration: Follow the [link](https://www.mathworks.com/help/vision/ref/cameracalibrator-app.html)
- Camera-LIDAR Calibration: Follow the [link](https://www.mathworks.com/help/lidar/ug/get-started-lidar-camera-calibrator.html).