### Checkout `rerun_viz` branch for new/better Rerun based visualization

Online visualization tool for [Swarm-SLAM](https://github.com/MISTLab/Swarm-SLAM)

# Installation
- Build the visualization tool in a ROS 2 workspace with `colcon build`.
- Install the [Zenoh ROS 2 DDS bridge](https://github.com/eclipse-zenoh/zenoh-plugin-ros2ddss) for ROS 2 to avoid flooding the network with discovery messages and unwanted communication.
- If you don't want to use Zenoh, run the following commands without the `ROS_DOMAIN_ID`

# Run the visualization station
```
export ROS_DOMAIN_ID=100 
ros2 launch cslam_visualization visualization_lidar.launch.py
```
# Run Zenoh
```
export ROS_DOMAIN_ID=100 
zenoh-bridge-ros2dds
```

Then run Swarm-SLAM on the robots.
