#!/usr/bin/env python

"""
Converts ARUCO's testdata .avi to a ROS bag
"""

import os
import sys
import time
import yaml

import numpy as np
import cv2

import rospy
import rosbag
import cv_bridge
from sensor_msgs.msg import CameraInfo

# Constructor to load OpenCV matrix from yaml file
# This is taken from: http://stackoverflow.com/a/15942429
def opencv_matrix_constructor(loader, node):
    mapping = loader.construct_mapping(node, deep=True)
    mat = np.array(mapping["data"])
    mat.resize(mapping["rows"], mapping["cols"])
    return mat


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print "Usage: {} FOLDER_WITH_TEST_DATA".format(sys.argv[0])
        exit()

    yaml.add_constructor(u"tag:yaml.org,2002:opencv-matrix", opencv_matrix_constructor)

    cap = cv2.VideoCapture(os.path.join(sys.argv[1], 'video.avi'))
    fps = cap.get(cv2.CAP_PROP_FPS)
    print "Opened video @ {} fps".format(fps)

    calib_text = open(os.path.join(sys.argv[1], 'intrinsics.yml')).read()
    # yml fixes
    calib_text = calib_text.replace('%YAML:1.0\n', '')
    calib_data = yaml.load(calib_text)

    bag_filename = os.path.basename(sys.argv[1].rstrip('/')) + '.bag'
    print "Saving output to '{}'".format(bag_filename)

    bridge = cv_bridge.CvBridge()
    # rospy.Time.now() requires a running roscore
    initial_time = time.time()
    seq = 0

    caminfo_msg = None

    with rosbag.Bag(bag_filename, 'w') as bag:

        while(cap.isOpened()):
            ret, frame = cap.read()

            if ret:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                msg = bridge.cv2_to_imgmsg(gray, "mono8")
                msg.header.seq = seq
                msg.header.stamp = rospy.Time.from_sec(initial_time + 1./fps * seq)
                bag.write('/camera/image_raw', msg, msg.header.stamp)

                if not caminfo_msg:
                    caminfo_msg = CameraInfo()
                    caminfo_msg.height, caminfo_msg.width, _ = np.shape(frame)
                    caminfo_msg.distortion_model = "plumb_bob"
                    caminfo_msg.D = calib_data['distortion_coefficients'].flatten()
                    caminfo_msg.K = calib_data['camera_matrix'].flatten()

                caminfo_msg.header = msg.header
                bag.write('/camera/camera_info', caminfo_msg, caminfo_msg.header.stamp)

                seq += 1
            else:
                break


    cap.release()
