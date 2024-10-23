# Franka Interactive Controllers


This repo is adapted from [Franka Interactive Controllers](https://github.com/nbfigueroa/franka_interactive_controllers/tree/main). It is set to run with ROS1 (tested with ROS Noetic). You will need `franka_ros` and `libfranka`. You need to have the package `franka_zed_gazebo` to launch in simulation (the launch file is getting the world and the robot description from that package, if you want to launch the robot wihtout the zed camera, then just edit the robot description):

```
roslaunch franka_interactive_controllers simulation_franka_teleop.launch
```

