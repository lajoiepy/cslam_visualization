# Swarm-SLAM visualization

You first will need to install Rerun: https://www.rerun.io/

Then in one terminal, execute:
```
rerun
```

In second terminal, execute:
```
sudo apt install python3-vcstool
git clone https://github.com/lajoiepy/cslam_visualization.git
cd cslam_visualization
git checkout rerun_viz
vcs import src < swarmslam_visualization.repos
cd docker/devel/
make build
make run
make attach
colcon build
ros2 launch your_visualization_launch_file.py
```

Then launch the Swarm-SLAM stack normally, and you should see the map appear in Rerun!
