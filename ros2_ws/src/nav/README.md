Required:

&#x20; Ubuntu 24.04

&#x20; ROS 2 Jazzy

&#x20; Gazebo Harmonic / gz sim

&#x20; ros-jazzy-ros-gz / ros\_gz\_bridge

&#x20; ros-jazzy-slam-toolbox

&#x20; ros-jazzy-tf2-ros

&#x20; python3-numpy

&#x20; nav\_stack.py

&#x20; nav\_explorer.py

&#x20; odom\_to\_tf.py

&#x20; compatible Gazebo world

&#x20; matching slam\_toolbox params YAML



Optional:

&#x20; rviz2

&#x20; YOLO Docker stack publishing /detections/found



Run with:

python3 nav\_stack.py \\

&#x20; --scan-topic /lidar2 \\

&#x20; --map-topic /map \\

&#x20; --cmd-vel-topic /model/vehicle\_blue/cmd\_vel \\

&#x20; --odom-topic /model/vehicle\_blue/odometry \\

&#x20; --clock-topic /clock \\

&#x20; --map-frame map \\

&#x20; --odom-frame vehicle\_blue/odom \\

&#x20; --base-frame vehicle\_blue/chassis \\

&#x20; --lidar-frame vehicle\_blue/lidar\_link/gpu\_lidar \\

&#x20; --found-topic /detections/found \\

&#x20; --slam-params-file \~/mapper\_params\_vehicle\_blue.yaml \\

&#x20; --use-sim-time true



Args can be changed to match other topics/robots/structures

Listed args are the defaults

