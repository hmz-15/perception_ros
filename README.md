# perception_ros


This repo contains two ros packages:

- **perception_ros** : useful ROS nodes for robot perception
    
- **seg_msgs** : ros message for image segmentation (panoptic, semantic, instance segmentation)

The perception_ros package contains two ROS nodes:

- **perception_node** for image processing and rquesting segmentation

- **pc_seg_generator_node** for semantic/geometric point cloud segment generation

## 1. Usage

### 1.1 Dependencies

- Ubuntu 16.04 (ROS Kinetic) or 18.04 (ROS Melodic) or Ubuntu 20.04 (ROS noetic)
- OpenCV 3 or 4
- PCL provided as catkin package using wstool
```
cd <your-ros-ws>
wstool init src
cd src
wstool merge -t . perception_ros/perception_ros_https.rosinstall
wstool update
```
### 1.2 Build
```
catkin build perception_ros
source devel/setup.bash
```
### 1.3 Run nodes
Please refer to `launch/robot_perception.launch` for how to run the nodes.
```
roslaunch perception_ros robot_perception.launch
```

## 2. perception_node

The node is implemented in `scripts/perception_node.py`. The node has two functions:

- **Image processing** : take in ROS topics of RGB image, depth image and camera info), and output processed topics

- **Segmentation request** : send RGB images to a [python3-based image segmentation server](https://github.com/hmz-15/Interactive-Scene-Reconstruction/tree/main/mapping/rp_server) and receive results via TCP communication, unpack the results and publish as `seg_msgs`


## 3. pc_seg_generator_node

The node is implemented in `src/pc_seg_generator_node.cpp`. It is majorly designed for generating per-frame point cloud segments in various ways given RGBD streams and the predicted segmentation results (mostly panoptic segmentation) published by `perception_node`. Some provided features are listed below:

- **depth segmentation** : in `src/depth_segmentation.cc` , adapted from the [ETH depth segmentation package](https://github.com/ethz-asl/depth_segmentation) with config file `cfg/seg_param.yaml`

- **point cloud processing and segmentation** : implemented in `src/pc_processing.cc` based on Point Cloud Library, config file can be found as `cfg/seg_param.yaml`

- **use depth/pc segmentation to refine panoptic segmentation** : use `cfg/pano_class.yaml` to specify panoptic classes accounted, and use `cfg/pc_gen_param.yaml` to specify options

Details for combining geometric and panoptic segmentation to generate per-frame point cloud segments can be found in our [ICRA paper](https://arxiv.org/pdf/2103.16095.pdf) 
