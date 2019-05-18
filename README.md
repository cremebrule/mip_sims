# mip_sims

Simple Mobile Inverted Pendulum (MIP) simulation, to be run in ROS1 and ROS2 described below:

# frtn01 - ROS1
Package to be run in ROS1. Add by copying the folder (and its contents) into your catkin_ws/src/ folder, and then run the following commands to build in your base catkin_ws folder (assuming your ROS workspace has already been sourced in your setup):
```
catkin_build --pkg frtn01
source devel/setup.bash
```

The simulation can then be run using roslaunch:

```
roslaunch frtn01 mip_sim.launch freq:={value} noise:={value}
```

Frequency = Plant Sampling Time, in seconds (I've run it at 0.001 - 0.005).
Noise = Max Noise Value, in the range +/- 0.5*{value} radians (I've run it at 0 - 0.05).

The resulting output file is then automatically saved to /home/{your_username}/.ros/test.txt, which can then be read and post processed by MATLAB.

# initial_test - ROS2
Package to be run in ROS2. First, add by copying the folder (and its contents) into your ros2_ws/src/ros2/ folder, and then download the following bitbucket (https://bitbucket.org/E-mil/realtime_project/src/master/) and copy it to your ros2_ws/src/ folder. Then, run the following commands to build in your base ros2_ws folder (assuming your ROS2 workspace has already been sourced in your setup):
```
colcon build --packages-select test_messages initial_test
. install/setup.bash
```

The simulation can then be run using launch:

```
ros2 launch initial_test mip_sim.launch.py __args:={path to your ros2_ws}/src/ros2/initial_test/plant_params.yaml
```

The simulation parameters in plant_params.yaml can be adjusted and run without having to rebuild the package.
Frequency = Plant Sampling Time, in milliseconds (I've run it at 1 - 5).
Noise = Max Noise Value, in the range +/- 0.5*{value} radians (I've run it at 0 - 0.05).
Timeout = Simulation time range, in seconds (I've run it at 5).

The resulting output file is then automatically saved to the folder from which you launched the simulation as test2.txt, which can then be read and post processed by MATLAB.
