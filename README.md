Online visualization tool for Swarm-SLAM

# Installation
- Build the visualization tool in a ROS 2 workspace with `colcon build`.
- Install the [Zenoh DDS bridge](https://github.com/eclipse-zenoh/zenoh-plugin-dds) for ROS 2 to avoid flooding the network with discovery messages and unwanted communication.

# Run the visualization station
ROS_DOMAIN_ID=100 ros2 launch cslam_visualization realsense_visualization.launch.py
zenoh-bridge-dds -d 100 --allow /cslam/.*

# Run Swarm-SLAM on the robots realsense example
ROS_DOMAIN_ID=$ROBOT_ID ros2 launch cslam_experiments experiment_realsense.launch.py robot_id:=$ROBOT_ID
zenoh-bridge-dds -d $ROBOT_ID --allow /cslam/.*
