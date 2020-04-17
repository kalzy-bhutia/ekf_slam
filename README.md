[![Build Status](https://travis-ci.org/sanjaythakur/ekf_slam.svg?branch=master)](https://travis-ci.org/sanjaythakur/ekf_slam)

This repository contains code for ekf-slam for educational purposes. A part of the code is taken from a comprehensive github repository on robot algorithms [here](https://github.com/AtsushiSakai/PythonRobotics).

- SLAM
SLAM stands for *Simultaneous Localization And Mapping*. It is a state-estimation problem problem where the state that has to be estimated is a joint of both the robot pose and the poses of all the landmarks together.
- EKF-SLAM
    - EKF-SLAM employs Extended Kalman Filter to estimate the unknown state of a state estimation problem. 
    - As this repo is not meant to be integrated with a real hardware, we simulate generation of observations, and the movement of robot.
- Toy problem description
    - This repository has an implementation of the EKF-SLAM.
    - Landmark pose has two dimensions
- How to use:
    - Check simulation.ipynb

*Points to note*:
- Note that anything *X* would be the state vector and *x* would be the robot pose.
- A robot has
    - exteroceptive_sensors: eg, a camera, a laser scanner, a sonar
    - proprioceptive_sensors: eg, wheel encoders, accelerometers, gyrometers
- We are explicitly adding noises to the readings from the proprioceptive sensors (u) and exteroceptive sensors (z). In the real robots, we won't add these noises explicitly. 
- We also simulate the readings from the sensor by manually keeping track of the true pose of the robot (xTrue) and the landmark positions. These are again not realistic and the corresponding readings in the real-world will come from appropriate exteroceptive sensors.

*Aside information*:
- Different sensors have different characteristics on their range and bearing. Here is a quick overview:
    - MonocularCamera --> range:bad, bearing:good
    - LaserRangeFinder --> range:good, bearing:good
    - Sonar --> range:good, bearing:bad
    - RGBD --> range:good, bearing:good
    - ARVA --> range:bad, bearing:bad
    - RFIDAntenna --> range:bad, bearing:bad