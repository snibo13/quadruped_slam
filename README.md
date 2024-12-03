# Extending Super Odometry with Joint State Measurements

## Description
This paper aims to enhance the Super Odometry framework by incorporating joint odometry constraints,
which are typically already integrated into the robot’s system.
Specifically, we can utilize kinematic data from the robot’s
joints to improve the accuracy and robustness of simultaneous
localization and mapping (SLAM). Moreover, Super Odometry
combines visual, LiDAR, and inertial sensors to address issues
in environments, and our approach further strengthens this
by incorporating additional velocity constraints derived from
joint encoder measurements. We demonstrate the method’s
effectiveness in a simulated environment with a Ghost Robotics
Spirit 40 quadruped robot.

## Dependencies
Ubuntu 20.04, ROS Noetic, GTSAM 4.2, Gazebo9, Quad-SDK

## Install
1) Install GTSAM
```bash
$ git clone https://github.com/borglab/gtsam.git
$ git checkout release/4.2
$ mkdir build
$ cd build
$ cmake ..
$ make check (optional, runs unit tests)
$ make install
```

2) Clone Repo
```bash
$ git clone https://github.com/snibo13/quadruped_slam.git
$ git checkout devel
$ catkin build
```

3) Download and Copy Dataset
```bash
$ cp -r /path/to/dataset ~/catkin_ws/src/quadruped_slam/dataset
```

4) Run Mapping Code
```bash
$ cd catkin_ws
$ roslaunch gtsam_bag play_bag_rviz.launch
```

Future Work: 
1) Update the code with LIO and VIO Constraints
2) Figure out Pose Intialization, Priors
3) Implement TF Conversion to Convert Measurements from Sensor Frame to IMU Frame 
4) Add a Header File cause this code is nasty rn