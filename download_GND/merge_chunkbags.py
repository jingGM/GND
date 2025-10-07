#!/usr/bin/env python
import argparse
import os
import re
import rosbag
import rospy

def merge_rosbag_chunks(chunks_dir, output_bag_path):

    bag_files = []
    for filename in os.listdir(chunks_dir):
        if filename.endswith('.bag') and '_chunk' in filename:
            bag_files.append(filename)
            print(filename)
    
    if not bag_files:
        rospy.logwarn("No chunked bag files found in the provided directory.")
        return


    # Example filename: ROSBAGNAME_chunk01.bag, ROSBAGNAME_chunk02.bag, etc.
    def get_chunk_number(file_name):
        match = re.search(r'_chunk(\d+)\.bag', file_name)
        if match:
            return int(match.group(1))
        else:
            return float('inf') 

    bag_files.sort(key=get_chunk_number)


    seen_messages = set()

    rospy.loginfo("Merging the following chunk files:")
    for f in bag_files:
        rospy.loginfo("  " + f)


    with rosbag.Bag(output_bag_path, 'w') as outbag:
        for bag_file in bag_files:
            bag_path = os.path.join(chunks_dir, bag_file)
            rospy.loginfo("Processing: " + bag_path)
            with rosbag.Bag(bag_path, 'r') as inbag:
                for topic, msg, t in inbag.read_messages():
                    key = (topic, t.secs, t.nsecs)
                    if key in seen_messages:
                        continue
                    seen_messages.add(key)
                    outbag.write(topic, msg, t)
    
    rospy.loginfo("Merged rosbag saved as: " + output_bag_path)

if __name__ == '__main__':
    rospy.init_node('rosbag_merger', anonymous=True)

    parser = argparse.ArgumentParser(description='process rosbags')
    parser.add_argument('--chunks_directory', type=str, required=True, default="")
    parser.add_argument('--merged_bag_file', type=str, required=True, default="")
    arguments = parser.parse_args()

    # Change these paths as needed.
    # chunks_directory = '/home/dibyendu/Desktop/GND/Chunk_bags/temp'      # Directory containing chunked rosbags
    # merged_bag_file = '/home/dibyendu/Desktop/GND/Chunk_bags/temp/GWU.bag'       # Path for the merged output bag

    merge_rosbag_chunks(arguments.chunks_directory, arguments.merged_bag_file)
