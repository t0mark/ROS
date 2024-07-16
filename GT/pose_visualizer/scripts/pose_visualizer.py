#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import os

class PoseVisualizer:
    def __init__(self, gt_file_path):
        self.pc_pub = rospy.Publisher('/visualization_pointcloud', PointCloud2, queue_size=10)
        self.gt_file_path = gt_file_path
        self.points = []
        self.read_gt_file()

    def read_gt_file(self):
        if not os.path.isfile(self.gt_file_path):
            rospy.logerr("GT file does not exist: {}".format(self.gt_file_path))
            return

        with open(self.gt_file_path, 'r') as file:
            lines = file.readlines()

        for line in lines:
            pose_data = line.strip().split()
            if len(pose_data) == 8:  # filename, x, y, z, qw, qx, qy, qz
                x = float(pose_data[1])
                y = float(pose_data[2])
                z = float(pose_data[3])
                self.points.append([x, y, z])

    def publish_pointcloud(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]

        pointcloud = pc2.create_cloud(header, fields, self.points)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            pointcloud.header.stamp = rospy.Time.now()
            self.pc_pub.publish(pointcloud)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pose_visualizer')
    gt_file_path = rospy.get_param('~gt_file')
    if not gt_file_path:
        rospy.logerr("Parameter gt_file is not set.")
        exit(1)
    visualizer = PoseVisualizer(gt_file_path)
    visualizer.publish_pointcloud()

