#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.num = 1

        self.pose_x = 0
        self.pose_y = 0
        self.pose_z = 0
        self.ori_x = 0
        self.ori_y = 0
        self.ori_z = 0
        self.ori_w = 0

        self.odom_sub = message_filters.Subscriber('/integrated_to_init', Odometry)
        self.image_sub = message_filters.Subscriber('/zed/left/image_rect_color/compressed', CompressedImage)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.image_sub], 10, 0.5)
        self.ts.registerCallback(self.callback)

        self.f = open('/home/tomark/toy/GT/train_data/train_meta_data.txt', 'w')

    def callback(self, odometry, compressedImage):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(compressedImage, desired_encoding="passthrough")

        image_num = str(self.num).zfill(5)
        image_name = os.path.join('/home/tomark/toy/GT/train_data', 'frame' + image_num + '.png')
        cv2.imwrite(image_name, cv_image)

        self.pose_x = odometry.pose.pose.position.x
        self.pose_y = odometry.pose.pose.position.y
        self.pose_z = odometry.pose.pose.position.z
        self.ori_x = odometry.pose.pose.orientation.x
        self.ori_y = odometry.pose.pose.orientation.y
        self.ori_z = odometry.pose.pose.orientation.z
        self.ori_w = odometry.pose.pose.orientation.w

        data = "frame" + image_num + ".png" + " " + str(self.pose_x) + " " + str(self.pose_y) + " " + str(self.pose_z) + " " + str(self.ori_x) + " " + str(self.ori_y) + " " + str(self.ori_z) + " " + str(self.ori_w) + "\n"

        self.f.write(data)
        self.f.flush()
        self.num += 1

def main(args=None):
    rospy.init_node('image_saver', anonymous=True)
    ImageSaver()
    rospy.spin()

if __name__ == '__main__':
    if not os.path.exists("/home/tomark/toy/GT/train_data/frame"):
        os.makedirs("/home/tomark/toy/GT/train_data/frame")
    if not os.path.exists("/home/tomark/toy/GT/train_data/train_meta_data.txt"):
        open("/home/tomark/toy/GT/train_data/train_meta_data.txt", 'w').close()
    main()

