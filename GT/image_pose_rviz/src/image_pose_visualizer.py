#!/usr/bin/env python

import rospy
import os
import cv2
import torch
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud
from geometry_msgs.msg import Point32
from torchvision import transforms, models
from model import model_parser, ResNetSimple

class ImagePoseVisualizer:
    def __init__(self):
        rospy.loginfo("Initializing ImagePoseVisualizer")

        # 모델 로드
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        rospy.loginfo(f"Using device: {self.device}")

        # 기존 모델 구조에 맞게 ResNetSimple 모델을 정의합니다.
        self.model = ResNetSimple(models.resnet34(pretrained=False)).to(self.device)
        rospy.loginfo("Model parsed successfully")

        self.model.load_state_dict(torch.load('/home/tomark/catkin_ws/src/image_pose_rviz/src/best_net.pth', map_location=self.device))
        rospy.loginfo("Model loaded successfully")

        self.model.eval()

        self.image_folder = rospy.get_param('~image_folder', '/home/tomark/toy/GT/train_frame')
        if not os.path.exists(self.image_folder):
            rospy.logerr(f"Image folder does not exist: {self.image_folder}")
            return
        self.image_files = sorted([f for f in os.listdir(self.image_folder) if os.path.isfile(os.path.join(self.image_folder, f))])
        if not self.image_files:
            rospy.logerr(f"No images found in folder: {self.image_folder}")
            return

        self.bridge = CvBridge()
        self.pointcloud_pub = rospy.Publisher('/visualization_position', PointCloud, queue_size=10)
        self.image_index = 0
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback)

        self.previous_points = []  # 이전 포인트들을 저장하기 위한 리스트

        rospy.loginfo("ImagePoseVisualizer initialized successfully")

    def timer_callback(self, event):
        if self.image_index >= len(self.image_files):
            self.image_index = 0

        image_path = os.path.join(self.image_folder, self.image_files[self.image_index])
        cv_image = cv2.imread(image_path)
        rospy.loginfo(f"Processing image: {image_path}")
        pose = self.extract_pose_from_image(cv_image)

        if pose is not None:
            self.publish_pointcloud(pose)
        else:
            rospy.logwarn(f"Pose extraction failed for image: {image_path}")

        self.image_index += 1

    def extract_pose_from_image(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (224, 224))
        image = transforms.ToTensor()(image).unsqueeze(0).to(self.device)

        with torch.no_grad():
            position, rotation, _ = self.model(image)  # 세 번째 반환 값을 무시

        pose = {
            'position': {'x': position[0, 0].item(), 'y': position[0, 1].item(), 'z': position[0, 2].item()},
            'orientation': {'x': rotation[0, 0].item(), 'y': rotation[0, 1].item(), 'z': rotation[0, 2].item(), 'w': rotation[0, 3].item()}
        }
        rospy.loginfo(f"Pose extracted: {pose}")
        return pose

    def publish_pointcloud(self, pose):
        pointcloud = PointCloud()
        pointcloud.header.frame_id = "base_link"
        pointcloud.header.stamp = rospy.Time.now()

        # 현재 포즈의 위치를 Point32로 추가
        position_point = Point32()
        position_point.x = pose['position']['x']
        position_point.y = pose['position']['y']
        position_point.z = pose['position']['z']
        self.previous_points.append(position_point)  # 이전 포인트 리스트에 추가

        # 이전 포인트들을 모두 PointCloud에 추가
        for point in self.previous_points:
            pointcloud.points.append(point)

        self.pointcloud_pub.publish(pointcloud)
        rospy.loginfo("Published pointcloud with {} points".format(len(self.previous_points)))

if __name__ == '__main__':
    rospy.init_node('image_pose_visualizer')
    visualizer = ImagePoseVisualizer()
    rospy.spin()

