#!/usr/bin/env python3

########################################################################################
# pointcloud to depth image conversion
# author: TPODAvia
# time:   14/06/2023
########################################################################################

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2

def convert_point_cloud_to_depth_image(point_cloud):
    # Extract x, y, z coordinates from the point cloud
    xyz_gen = pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True)

    # TODO: Replace with your camera intrinsics
    fx, fy = 525.0, 525.0  # Focal lengths
    cx, cy = 319.5, 239.5  # Optical centers

    # Initialize depth image
    depth_image = np.zeros((480, 640), dtype=np.float32)

    for point in xyz_gen:
        x, y, z = point
        if z > 0:
            u = int(fx * x / z + cx)
            v = int(fy * y / z + cy)
            
            if 0 <= u < depth_image.shape[1] and 0 <= v < depth_image.shape[0]:
                depth_image[v, u] = z

    return depth_image

def point_cloud_callback(msg):
    depth_image = convert_point_cloud_to_depth_image(msg)
    depth_image_msg = bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
    depth_image_msg.header = msg.header
    depth_image_pub.publish(depth_image_msg)

if __name__ == "__main__":
    rospy.init_node("point_cloud_to_depth_image")
    bridge = CvBridge()

    # Subscribe to the point cloud topic
    rospy.Subscriber("/rslidar_points", PointCloud2, point_cloud_callback)

    # Publish the depth image
    depth_image_pub = rospy.Publisher("/depth_image", Image, queue_size=10)

    rospy.spin()
