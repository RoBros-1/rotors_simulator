RotorS
===============

RotorS is a MAV gazebo simulator.
It provides some multirotor models such as the [AscTec Hummingbird](http://www.asctec.de/en/uav-uas-drone-products/asctec-hummingbird/), the [AscTec Pelican](http://www.asctec.de/en/uav-uas-drone-products/asctec-pelican/), or the [AscTec Firefly](http://www.asctec.de/en/uav-uas-drone-products/asctec-firefly/), but the simulator is not limited for the use with these multicopters.

There are simulated sensors coming with the simulator such as an IMU, a generic odometry sensor, and the [VI-Sensor](http://wiki.ros.org/vi_sensor), which can be mounted on the multirotor.

This package also contains some example controllers, basic worlds, a joystick interface, and example launch files.

Below we provide the instructions necessary for getting started. See RotorS' wiki for more instructions and examples (https://github.com/ethz-asl/rotors_simulator/wiki).

If you are using this simulator within the research for your publication, please cite:
```bibtex
@Inbook{Furrer2016,
author="Furrer, Fadri
and Burri, Michael
and Achtelik, Markus
and Siegwart, Roland",
editor="Koubaa, Anis",
chapter="RotorS---A Modular Gazebo MAV Simulator Framework",
title="Robot Operating System (ROS): The Complete Reference (Volume 1)",
year="2016",
publisher="Springer International Publishing",
address="Cham",
pages="595--625",
isbn="978-3-319-26054-9",
doi="10.1007/978-3-319-26054-9_23",
url="http://dx.doi.org/10.1007/978-3-319-26054-9_23"
}

# Installation Instructions - ROS Noetic

```
sudo apt-get update
```

Clone this dependency into `catkin_ws/src`:
```
cd ~/catkin_ws/src
git clone https://github.com/RoBros-1/mav_comm.git
```

Install other dependencies:
```
sudo apt-get install ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink protobuf-compiler libgoogle-glog-dev ros-noetic-control-toolbox ros-noetic-mavros
```

Build everything:
```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Run Instructions

Launch the firefly in gazebo:
```
roslaunch rotors_gazebo mav.launch
```

To make the firefly move:
```
rostopic pub /firefly/command/motor_speed mav_msgs/Actuators '{angular_velocities: [0, 0, 0, 0, 0, 0]}'
rostopic pub /firefly/command/motor_speed mav_msgs/Actuators '{angular_velocities: [100, 100, 100, 100, 100, 100]}'
rostopic pub /firefly/command/motor_speed mav_msgs/Actuators '{angular_velocities: [1000, 1000, 1000, 1000, 1000, 1000]}'
```
