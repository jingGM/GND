import copy
from scipy.spatial.transform import Rotation
from rosbag import Bag
import numpy as np
import argparse

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped


class BagProcessor:
    def __init__(self):
        self.plane_threshold = 0.3
        self.imu_odom_time_threshold = 0.01
        np.random.seed(0)
        self.odom_topics = []
        self.odom_timestamps = []

    def get_topic_info(self, bag_dir, odom_name):
        with Bag(bag_dir, 'r') as read_bag:
            for topic, msg, t in read_bag.read_messages(topics=odom_name):
                self.odom_topics.append(msg)
                self.odom_timestamps.append(msg.header.stamp.to_sec())

    def create_new_rosbag(self, previous_dir, output_dir, imu_name=None, pose_name=None,
                          odom_name="/odometry/filtered"):
        write_bag = Bag(output_dir, 'w')
        read_bag = Bag(previous_dir, 'r')

        for topic, msg, t in read_bag.read_messages():
            if imu_name is not None and topic == imu_name:
                msg = self.process_imu_message(msg=msg)
            if pose_name is not None and topic == odom_name:
                new_msg = self.process_pose(msg=msg)
                write_bag.write(topic=pose_name, t=t, msg=new_msg)
            write_bag.write(topic=topic, t=t, msg=msg)
        write_bag.close()

    def process_pose(self, msg:Odometry):
        new_msg = PoseWithCovarianceStamped()
        new_msg.header = copy.deepcopy(msg.header)
        new_msg.pose = copy.deepcopy(msg.pose)
        return new_msg

    def process_imu_message(self, msg: Imu):
        message_time = msg.header.stamp.to_sec()
        if message_time <= self.odom_timestamps[0]:
            msg.orientation = self.odom_topics[0].pose.pose.orientation
        elif message_time >= self.odom_timestamps[-1]:
            msg.orientation = self.odom_topics[-1].pose.pose.orientation
        else:
            big_idx = next(x for x, val in enumerate(self.odom_timestamps) if val >= message_time)
            big_time = self.odom_timestamps[big_idx]
            small_time = self.odom_timestamps[big_idx - 1]
            big_raw_value = self.odom_topics[big_idx]
            small_raw_value = self.odom_topics[big_idx - 1]
            big_value = Rotation.from_quat(
                [big_raw_value.pose.pose.orientation.x, big_raw_value.pose.pose.orientation.y,
                 big_raw_value.pose.pose.orientation.z, big_raw_value.pose.pose.orientation.w]).as_euler(seq='xyz')
            small_value = Rotation.from_quat(
                [small_raw_value.pose.pose.orientation.x, small_raw_value.pose.pose.orientation.y,
                 small_raw_value.pose.pose.orientation.z, small_raw_value.pose.pose.orientation.w]).as_euler(seq='xyz')
            current_rpy = small_value + (big_value - small_value) * (message_time - small_time) / (
                        big_time - small_time)
            current_quat = Rotation.from_euler(angles=current_rpy, seq="xyz").as_quat()
            msg.orientation.x = current_quat[0]
            msg.orientation.y = current_quat[1]
            msg.orientation.z = current_quat[2]
            msg.orientation.w = current_quat[3]
        return msg


def get_args():
    parser = argparse.ArgumentParser(description='process rosbags')
    parser.add_argument('--input_rosbag', type=str, required=True, default="")
    parser.add_argument('--output_rosbag', type=str, required=True, default="")
    parser.add_argument('--odom_topic', type=str, default="/odometry/filtered")
    parser.add_argument('--imu_topic', type=str, default="/camera/imu")
    return parser.parse_args()


if __name__ == "__main__":
    processor = BagProcessor()
    args = get_args()
    processor.get_topic_info(bag_dir=args.input_rosbag, odom_name=args.odom_topic)
    processor.create_new_rosbag(previous_dir=args.input_rosbag, output_dir=args.output_rosbag, imu_name=args.imu_topic)
