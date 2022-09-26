# Livox-Split-Mid-100

A ROS package that does the same as setting "multi_topic" on a Livox Mid-100 device, but does so even when the data is already recorded and you forgot to set the "multi_topic" parameter in the first place.

![image_color_topics](https://github.com/fallow24/Livox-Split-Mid-100/blob/main/splitlivox.png)

The Mid-100 is actually a set of calibrated Mid-40 sensors, thus each message corresponds to one device.
This program figures out which device the message corresponds to, and publishes it in another topic.

## Prerequisites

Install ROS and setup a catkin_ws. 
Depending on your installation, the **name of ROS distribution** in the next dependencies might change:
```bash
sudo apt install ros-noetic-pcl-ros ros-noetic-pcl-msgs ros-noetic-pcl-conversions
```

If you experience any problems at compile time, try installing these dependencies:
```bash
sudo apt install libpcl-dev libpcl-conversions-dev 
```

## How to install
```bash
cd ~/catkin_ws/src
git clone https://github.com/fallow24/Livox-Split-Mid-100.git
cd ..
catkin_make
```

## How to use
```bash
rosrun livox_split_mid100 livox_split_mid100
```
The program will by default read data from **/livox/lidar** and publish to:
 - /livox/lidar_l (left device)
 - /livox/lidar_c (center device)
 - /livox/lidar_r (right device)
 
You can load the *livox_split_view.rviz* config file in RVIZ to vizualize the topics as in the image above, to make sure everything works.

