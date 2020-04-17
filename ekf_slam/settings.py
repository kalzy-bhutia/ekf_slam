"""
author: Sanjay Thakur (@WanabeMarkovian)
blog: sanjaykthakur.com
"""
import numpy as np

# EKF state covariance. The state vector consitutes of x, y, and yaw - hence 3 elements.
Cx = np.diag([0.5, 0.5, np.deg2rad(30.0)]) ** 2

# Simulation parameter for measurement noise (exteroceptive sensors). 
# As a toy problem, we will use the laser data
# from LIDAR as measurement. Note that laser 
# data has both range and bearing properties.
# Hence 2 dimensions. Note that, this is just 
# a simulation parameter and should not exist 
# explicitly in the real-world. The measurements 
# from the real world exteroceptive sensors would automatically have noise 
# it. For the simulation, it's value should be 
# according to how much the programmer thinks 
# they should trust the measurements.
Qsim = np.diag([0.2, np.deg2rad(1.0)]) ** 2

# Simulation parameter for control process noise (proprioceptive sensors). 
# As a toy problem, we will use the linear and angular
# velocity as the control input. Hence 2 dimensions. 
# Note that, this is just 
# a simulation parameter and should not exist 
# explicitly in the real-world. The measurements 
# from the real world proprioceptive sensors would automatically have noise 
# it. For the simulation, it's value should be 
# according to how much the programmer thinks 
# they should trust the measurements.
Rsim = np.diag([1.0, np.deg2rad(10.0)]) ** 2

DT = 0.25  # time tick [s]
TOTAL_SIM_TIME = 10.  # simulation time [s]
ROBOT_STATE_DIMS = 3 # State size [x,y,yaw]
LM_STATE_DIMS = 2 # Landmark state size [x,y]
NEW_LM_THRESHOLD = 2. # Threshold of Mahalanobis distance for data association.
MAX_RANGE = 20.0  # maximum observation range

# True positions of the landmarks as RFID positions [x, y]
RFID = np.array([[10.0, -2.0],
                 [15.0, 10.0],
                 [3.0, 15.0],
                 [-5.0, 20.0]])