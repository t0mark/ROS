#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fstream>

std::ofstream pose_file;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (pose_file.is_open()) {
        pose_file << msg->header.stamp.toSec() << ","
                  << msg->pose.pose.position.x << ","
                  << msg->pose.pose.position.y << ","
                  << msg->pose.pose.position.z << ","
                  << msg->pose.pose.orientation.x << ","
                  << msg->pose.pose.orientation.y << ","
                  << msg->pose.pose.orientation.z << ","
                  << msg->pose.pose.orientation.w << std::endl;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_to_txt");
    ros::NodeHandle nh;

    // 파일 열기
    pose_file.open("pose_data.txt", std::ios::out);
    if (!pose_file) {
        ROS_ERROR("Failed to open file");
        return -1;
    }

    ros::Subscriber sub = nh.subscribe("/integrated_to_init", 1000, poseCallback);

    ros::spin();

    pose_file.close();
    return 0;
}

