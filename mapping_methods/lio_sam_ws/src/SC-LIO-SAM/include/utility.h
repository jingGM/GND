#pragma once

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <limits>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <sstream>

using namespace std;

typedef std::numeric_limits< double > dbl;

typedef pcl::PointXYZI PointType;

enum class SensorType { MULRAN, VELODYNE, OUSTER };

class ParamServer
{
public:

    ros::NodeHandle nh;

    std::string robot_id;

    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Velodyne Sensor Configuration: Velodyne
    SensorType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;

    // IMU
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance; 
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    
    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer()
    {
        nh.param<std::string>("/robot_id", robot_id, "roboat");

        nh.param<std::string>("sc_liosam/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("sc_liosam/imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>("sc_liosam/odomTopic", odomTopic, "odometry/imu");
        nh.param<std::string>("sc_liosam/gpsTopic", gpsTopic, "odometry/gps");

        nh.param<std::string>("sc_liosam/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("sc_liosam/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("sc_liosam/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("sc_liosam/mapFrame", mapFrame, "map");

        nh.param<bool>("sc_liosam/useImuHeadingInitialization", useImuHeadingInitialization, false);
        nh.param<bool>("sc_liosam/useGpsElevation", useGpsElevation, false);
        nh.param<float>("sc_liosam/gpsCovThreshold", gpsCovThreshold, 2.0);
        nh.param<float>("sc_liosam/poseCovThreshold", poseCovThreshold, 25.0);

        nh.param<bool>("sc_liosam/savePCD", savePCD, false);
        nh.param<std::string>("sc_liosam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

        std::string sensorStr;
        nh.param<std::string>("sc_liosam/sensor", sensorStr, "");
        if (sensorStr == "velodyne")
        {
            sensor = SensorType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            sensor = SensorType::OUSTER;
        }
        else if (sensorStr == "mulran")
        {
            sensor = SensorType::MULRAN;
        }
        else
        {
            ROS_ERROR_STREAM(
                "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'mulran'): " << sensorStr);
            ros::shutdown();
        }

        nh.param<int>("sc_liosam/N_SCAN", N_SCAN, 16);
        nh.param<int>("sc_liosam/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<int>("sc_liosam/downsampleRate", downsampleRate, 1);
        nh.param<float>("sc_liosam/lidarMinRange", lidarMinRange, 1.0);
        nh.param<float>("sc_liosam/lidarMaxRange", lidarMaxRange, 1000.0);

        nh.param<float>("sc_liosam/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("sc_liosam/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("sc_liosam/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("sc_liosam/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("sc_liosam/imuGravity", imuGravity, 9.80511);
        nh.param<float>("sc_liosam/imuRPYWeight", imuRPYWeight, 0.01);
        nh.param<vector<double>>("sc_liosam/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("sc_liosam/extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>("sc_liosam/extrinsicTrans", extTransV, vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        nh.param<float>("sc_liosam/edgeThreshold", edgeThreshold, 0.1);
        nh.param<float>("sc_liosam/surfThreshold", surfThreshold, 0.1);
        nh.param<int>("sc_liosam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>("sc_liosam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        nh.param<float>("sc_liosam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        nh.param<float>("sc_liosam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>("sc_liosam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

        nh.param<float>("sc_liosam/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>("sc_liosam/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<int>("sc_liosam/numberOfCores", numberOfCores, 2);
        nh.param<double>("sc_liosam/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("sc_liosam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>("sc_liosam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("sc_liosam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("sc_liosam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        nh.param<bool>("sc_liosam/loopClosureEnableFlag", loopClosureEnableFlag, false);
        nh.param<float>("sc_liosam/loopClosureFrequency", loopClosureFrequency, 1.0);
        nh.param<int>("sc_liosam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>("sc_liosam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>("sc_liosam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>("sc_liosam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>("sc_liosam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        nh.param<float>("sc_liosam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("sc_liosam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("sc_liosam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        usleep(100);
    }

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
    {
        sensor_msgs::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }
};


sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}


float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter = " ")
{
    // delimiter: ", " or " " etc.

    int precision = 3; // or Eigen::FullPrecision, but SCD does not require such accruate precisions so 3 is enough.
    const static Eigen::IOFormat the_format(precision, Eigen::DontAlignCols, delimiter, "\n");
 
    std::ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(the_format);
        file.close();
    }
}

std::string padZeros(int val, int num_digits = 6) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}
