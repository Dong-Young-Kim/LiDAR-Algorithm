# LiDAR-Sampling-Algorithm
for maintain the number of LiDAR points constant, regardless of distance

![Ubuntu](https://img.shields.io/badge/-Ubuntu-orange)
![ROS](https://img.shields.io/badge/-ROS-lightgrey)

## Operating Environment
- OS     : ubuntu 20.04
- ROS    : noetic (pcl required)
- sensor : Velodyne VLP-16 (16channel)

## How to use
1. Build package through the 'catkin_make' build system
2. to launch full process "roslaunch sampling_algorithm smp_test.launch"
3. change all parameters at 'launch/smp_test.launch'

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Dong-Young-Kim/LiDAR-Sampling-Algorithm.git
cd ..
catkin_make
roslaunch sampling_algorithm smp_test.launch
```
