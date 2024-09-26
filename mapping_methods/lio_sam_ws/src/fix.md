
### in include/utility.h
```
#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/opencv.hpp>
```

### in CMakeLists.txt
```
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

set(PYTHON_VERSION "3.8")
set(CONDA_ENV_DIR "/home/jing/anaconda3/envs/gn")
set(PYTHON_EXECUTABLE "${CONDA_ENV_DIR}/bin/python${PYTHON_VERSION}")
set(PYTHON_INCLUDE_DIR "${CONDA_ENV_DIR}/include/python${PYTHON_VERSION}")
set(PYTHON_LIB "${CONDA_ENV_DIR}/lib/libpython${PYTHON_VERSION}.so")
include_directories(${PYTHON_INCLUDE_DIR})
include_directories("/usr/local/include/gtsam/3rdparty/Eigen/")
```

### Depends on the version of GTSAM, in src/imuPreintegration.cpp
change
```
boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
```
to
```
std::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
```


### debug IMU: src/imageProjection.cpp:imuHandler() uncomment
```
cout << std::setprecision(6);
cout << "IMU acc: " << endl;
cout << "x: " << thisImu.linear_acceleration.x << 
      ", y: " << thisImu.linear_acceleration.y << 
      ", z: " << thisImu.linear_acceleration.z << endl;
cout << "IMU gyro: " << endl;
cout << "x: " << thisImu.angular_velocity.x << 
      ", y: " << thisImu.angular_velocity.y << 
      ", z: " << thisImu.angular_velocity.z << endl;
double imuRoll, imuPitch, imuYaw;
tf::Quaternion orientation;
tf::quaternionMsgToTF(thisImu.orientation, orientation);
tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
cout << "IMU roll pitch yaw: " << endl;
cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
```


## Errors

```
Message removed because it is too old:
set the frame to //camera_init in rviz
```


## Added 
in imageProjection.cpp and utility.h
```
extern const float sensorMinimumHeight = -100.0;


if (thisPoint.z < sensorMinimumHeight)
    continue;
```