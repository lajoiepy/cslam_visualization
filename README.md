
source ros2
source ws

# Visualization station
conda activate viz
ROS_DOMAIN_ID=2 ros2 launch cslam_visualization realsense_visualization.launch.py
zenoh-bridge-dds -d 2 --allow /cslam/.*

# Robots
conda activate cslam
ROS_DOMAIN_ID=0 ros2 launch cslam_experiments experiment_realsense.launch.py robot_id:=0
zenoh-bridge-dds -d 0 --allow /cslam/.*