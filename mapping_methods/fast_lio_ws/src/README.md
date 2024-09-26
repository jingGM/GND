# FAST-LIO and FAST-LIO-LOCALIZATION

## Dependencies

### FAST-LIO

1. Install Livox ROS Driver
```sh 
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
cd build && cmake ..
make
sudo make install
cd ../..

git clone https://github.com/Livox-SDK/livox_ros_driver.git 
```

2. Update submodule
```sh
cd FAST_LIO
git submodule update --init
cd ..
```
### FAST-LIO-LOCALIZATION
1. Update submodule 
```sh
cd FAST_LIO_LOCALIZATION
git submodule update --init
cd ..
```

2. Install dependencies
```sh
sudo apt install python-numpy python-yaml
pip install open3d pyrsistent
```

3. Install ros-numpy from source ([ref](https://github.com/eric-wieser/ros_numpy/pull/39))
```sh
git clone https://github.com/eric-wieser/ros_numpy.git
```

## Build

```sh
cd ..
catkin_make
source ./devel/setup.sh
```

## Run
### Mapping

Before running, change the path in [this line](https://github.com/jingGM/GND/blob/e1a7ec25de8b8d01193e7c506c692dcd51c0e495/mapping_methods/fast_lio_ws/src/SC-PGO/launch/fastlio_ouster64.launch#L33) to your workspace.

```sh
roslaunch fast_lio mapping_ouster64.launch
roslaunch aloam_velodyne fastlio_ouster64.launch
rosbag play PATH_TO_YOUR_BAG.bag
```

### Localization

```sh
roslaunch fast_lio_localization localization_ouster64.launch map:=PATH_TO_YOUR_MAP.pcd
rosbag play PATH_TO_YOUR_BAG.bag
```
Provide initial guess of the robot pose by using the '2D Pose Estimate' Tool in RVIZ.

First play the bag for about 0.5s, and then pause the bag until the initialization succeed.