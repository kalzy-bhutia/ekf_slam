"""
author: Sanjay Thakur (@WanabeMarkovian)
blog: sanjaykthakur.com
"""
from ekf_slam.settings import *
import math

def inverse_observation_model(s=None):
	pass


def direct_observation_model():
	"""
	The robot observes landmarks that had been previously mapped, 
	and uses them to correct both its self-localization and the 
	localization of all landmarks in space. In this case, 
	therefore, both localization and landmarks uncertainties decrease.
	"""
	pass


def motion_model(x, u):
	"""
	Computes the motion model based on current state and input function
	on reaching a new point of view of the scene. Due to unavoidable noise 
	and errors, this motion increases the uncertainty on the robot’s localization.
	
	:param x: (robot_state_dimensions x 1) pose estimation
	:param u: control information from a proprioceptive sensor, 2x1 control input for [v; w]

	:returns: the resulting state after the control function is applied
	"""
	F = np.array([[1.0, 0, 0],
				  [0, 1.0, 0],
				  [0, 0, 1.0]])

	B = np.array([[DT * math.cos(x[2, 0]), 0],
				  [DT * math.sin(x[2, 0]), 0],
				  [0.0, DT]])

	x = (F @ x) + (B @ u)
	return x