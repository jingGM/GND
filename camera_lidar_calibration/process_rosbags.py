import argparse
import copy

import open3d as o3d
import numpy as np
import cv2
import os
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from scipy.io import savemat
import os
import pickle
from os.path import join
import cv2
import ros_numpy
import numpy as np
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from rosbag import Bag
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, PointCloud2, CompressedImage, Image
from tqdm import tqdm
from cv_bridge import CvBridge, CvBridgeError


class ProcessROSbag:
    def __init__(self, lidar_topic="", camera_topic="", output_door="", compressed_image=False, only_camera=False):
        self.lidar_topic = lidar_topic
        self.camera_topic = camera_topic
        self.output_door = output_door
        self.only_camera = only_camera

        self.bridge = CvBridge()
        self.lidar_buffer_length = 2
        self.camera_buffer_length = 60  # according to the frenquency
        self.time_threshold = 0.015
        self.compressed_image = compressed_image

        self.lidar_buffer = []
        self.lidar_time_buffer = []
        self.camera_buffer = []
        self.camera_time_buffer = []
        self.index = 0
        self.current_time = 0

        self.folder_idx = 0

    def reset_buffers(self):
        self.lidar_buffer = []
        self.lidar_time_buffer = []
        self.camera_buffer = []
        self.camera_time_buffer = []
        self.index = 0

    def process_rosbags(self, rosbags=[""]):
        for bag_dir in rosbags:
            self.folder_idx += 1
            folder = join(self.output_door, str(self.folder_idx))
            self.reset_buffers()
            read_bag = Bag(bag_dir, 'r')

            for topic, msg, t in tqdm(read_bag.read_messages(), desc="{}: ".format(self.folder_idx)):
                self.current_time = t.to_sec()
                if topic == self.lidar_topic:
                    self.add_lidar(msg)
                    if len(self.lidar_buffer) > 1:
                        self.save_data(folder)
                elif topic == self.camera_topic:
                    self.add_camera(msg)

    def add_lidar(self, data: PointCloud2):
        pcl_data = list(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(pcl_data)

        if len(self.lidar_buffer) > self.lidar_buffer_length:
            self.lidar_buffer.pop(0)
            self.lidar_time_buffer.pop(0)
        self.lidar_buffer.append(cloud)
        self.lidar_time_buffer.append(self.current_time)
        # self.lidar_time_buffer.append(data.header.stamp.to_sec())

    def add_camera(self, data: Image):
        if self.compressed_image:
            np_arr = np.fromstring(data.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            try:
                image_np = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

        if len(self.camera_buffer) > self.camera_buffer_length:
            self.camera_buffer.pop(0)
            self.camera_time_buffer.pop(0)
        self.camera_buffer.append(image_np)
        self.camera_time_buffer.append(self.current_time)
        # self.camera_time_buffer.append(data.header.stamp.to_sec())

    def save_data(self, folder: str = None):
        lidar_data = self.lidar_buffer[-2]
        lidar_time = self.lidar_time_buffer[-2]

        camera_times = np.array(copy.deepcopy(self.camera_time_buffer))
        difference = np.abs(camera_times - lidar_time)
        min_idx = np.argmin(difference)

        diff = difference[min_idx]
        image = self.camera_buffer[min_idx]
        print("time difference: {}".format(diff))
        if diff < self.time_threshold:
            image_folder = join(folder, "image")
            lidar_folder = join(folder, "lidar")
            if not os.path.exists(lidar_folder):
                os.makedirs(lidar_folder)
            if not os.path.exists(image_folder):
                os.makedirs(image_folder)

            cv2.imwrite(join(image_folder, "file_{}.png".format(self.index)), image)
            o3d.io.write_point_cloud(join(lidar_folder, "file_{}.pcd".format(self.index)), lidar_data)
            self.index += 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='mapping')
    parser.add_argument('--lidar_topic', type=str, default="/ouster/points")
    parser.add_argument('--camera_topic', type=str, default="/camera_processed")
    parser.add_argument('--output_door', type=str, default="/media/jing/update/rosbag_calibration")
    parser.add_argument('--bag', type=str, default="/home/jing/Downloads/cam_lidar_calibration_sm.bag")
    parser.add_argument('--only_camera', action='store_true', default=False)
    parser.add_argument('--compress_image', action='store_true', default=False)
    args = parser.parse_args()

    rosbag = ProcessROSbag(lidar_topic=args.lidar_topic, camera_topic=args.camera_topic, output_door=args.output_door,
                           only_camera=args.only_camera, compressed_image=args.compress_image)
    rosbag.process_rosbags([args.bag])