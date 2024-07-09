# ROS
# 진행환경
- Ubuntu 20.04 LTS
- python 3.8.10
- ROS1 noetic
- catkin 0.8.10
- empy 3.3.4
- matplotlib 3.7.5
- numpy 1.24.4
- nvidia driver
- opencv-python 4.10.0.84
- pip 20.0.2
- tensorboard 2.14.0
- torch 2.3.1
- torchvision 0.18.1

# 진행과정
- PonseNet 학습
- LeGo-LOAM 설치
  
# PoseNet 학습실습
- PoseNet 폴더의 README.md 수행
- Train 결과
![PoseNet_train_result](https://github.com/t0mark/ROS/assets/128698845/ccbcad28-ad0a-498d-99f4-3f55dea55aba)
- Test 결과
![PoseNet_test_result](https://github.com/t0mark/ROS/assets/128698845/14b2e458-4bb4-4f07-8842-b375f66d62fe)

# LeGO-LOAM
- setting (Folder, CMakeLists, utility)
![SLAM_setting_folder](https://github.com/t0mark/ROS/assets/128698845/1c312686-cb3f-4877-82ba-228a335aff91)

![SLAM_setting_CMakeLists](https://github.com/t0mark/ROS/assets/128698845/fa817e78-21cc-4020-af44-298baf73f40a)

![SLAM_setting_utility h](https://github.com/t0mark/ROS/assets/128698845/fb0aa7db-5534-4997-a971-b11c6b661137)

- 사용법
  - Launch
  ![SLAM_pkg_launch](https://github.com/t0mark/ROS/assets/128698845/3e265b2d-f27c-4207-a8b8-ea58085d8248)

  - Datasets
  ![SLAM_rosbag_data](https://github.com/t0mark/ROS/assets/128698845/b31c5319-18e5-47f6-8157-0c31b53b7650)

  - Rviz
  ![SLAM_RViz](https://github.com/t0mark/ROS/assets/128698845/332ef32a-1efd-4919-ab9f-1733e36bd842)
