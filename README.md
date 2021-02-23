# GMapping

> G. Grisetti, C. Stachniss and W. Burgard, "Improved Techniques for Grid Mapping With Rao-Blackwellized Particle Filters," in IEEE Transactions on Robotics, vol. 23, no. 1, pp. 34-46, Feb. 2007, doi: 10.1109/TRO.2006.889486.

## Introduction

The most codes in this repository were adapted from [here](https://github.com/Wleisure95/laser_slam), while the original codes of gmapping can be found [here](https://github.com/ros-perception/openslam_gmapping). Note that most redundant codes in the original repository were not kept and I used more modern C++ to rewrite the codes in [Google style](https://google.github.io/styleguide/cppguide.html).

## Dependencies

- [ROS](http://wiki.ros.org/ROS/Installation)
- [glog](https://github.com/google/glog)

## Compilation

```bash
mkdir -p ~/laser_slam_ws/src
cd ~/laser_slam_ws/src/
git clone git@github.com:TongLing916/gmapping_clean.git
cd ..
catkin_make -j
```

## Execution

```bash
# Terminal 1
cd ~/laser_slam_ws
source devel/setup.bash
roslaunch gmapping gmapping_sim.launch
```

```bash
# Terminal 2
cd ~/laser_slam_ws/src/gmapping_clean/data
rosbag play --clock gmapping.bag
```

```bash
# Terminal 3
rosrun rviz rviz
```

## Demo

![](https://raw.githubusercontent.com/TongLing916/gmapping_clean/main/data/gmapping.PNG)
