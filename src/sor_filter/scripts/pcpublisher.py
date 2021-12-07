#!/usr/bin/env python3
# coding: utf-8
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

import pcl
import pcl_helper

from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.msg import BoundingBoxArray

def callback(msg):
    filtered_boxes = BoundingBoxArray()
    for box in msg.boxes:
        if box.dimensions.x==box.dimensions.y==box.dimensions.z==0.0:
            continue
        new_data = [box.pose.position.x,box.pose.position.y,box.pose.position.z]

    new_cloud = pcl.PointCloud(1)
    new_cloud.from_array(new_data)
    new_cloud = pcl_helper.XYZ_to_XYZRGB(new_cloud,[255,255,255])
    cloud_new = pcl_helper.pcl_to_ros(new_cloud) #PCL을 ROS 메시지로 변경     
    pub.publish(cloud_new)

if __name__ == "__main__":
    rospy.init_node('filterd_pcd', anonymous=True)
    rospy.Subscriber('/cluster_decomposer/boxes', BoundingBoxArray, callback, queue_size=1)
    pub = rospy.Publisher("/filtered_pc", PointCloud2, queue_size=1)

    rospy.spin()


