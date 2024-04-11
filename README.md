# Swarm-SLAM visualization

In first terminal:
```
sudo apt install python3-vcstool
git clone https://github.com/lajoiepy/cslam_visualization.git
cd cslam_visualization
vcs import src < swarmslam_visualization.repos
cd docker/
make build
make run
make attach
colcon build
ros2 launch your_visualization_launch_file.py
```
