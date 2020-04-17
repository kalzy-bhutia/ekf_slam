from ekf_slam.utilities import *

class EKF():
	def ekf_slam(self, X, P, u, z):
		"""
		Updates the state belief estimate using EKF.
		:param X: belief during the last time step
		:param P: covariance in the belief during the last time step
		:param u: the last control information
		:param z: measurement taken at the new state
		:returns: the next estimated state belief and associated covariance
		"""
		# predict based on the control information
		X, P, G, Fx = self.predict(X, P, u)
		# correct the belief based on the measurement
		X, P = self.update(X, P, u, z)
		return X, P
	
	def predict(self, X, P, u):
		"""
		Performs the prediction step of EKF SLAM
		
		:param X: nx1 state vector
		:param P: nxn covariance matrix
		:param u: 2x1 control vector
		:returns: predicted state vector, predicted covariance, jacobian of control vector, transition fx
		"""
		X[:ROBOT_STATE_DIMS] = motion_model(X[:ROBOT_STATE_DIMS], u)
		G, Fx = jacob_motion(X[:ROBOT_STATE_DIMS], u)
		P[:ROBOT_STATE_DIMS, :ROBOT_STATE_DIMS] = G.T @ P[:ROBOT_STATE_DIMS, :ROBOT_STATE_DIMS] @ G + Fx.T @ Cx @ Fx
		return X, P, G, Fx
	
	def update(self, X, P, u, z):
		"""
		Performs the update step of EKF SLAM
		
		:param X:  nx1 the predicted pose of the system and the pose of the landmarks
		:param P:  nxn the predicted covariance
		:param u:  2x1 the control function 
		:param z:  the measurements read at new position

		:returns:     the updated state and covariance for the system
		"""
		for lm_index in range(z.shape[0]):
			minid = search_correspond_LM_ID(X, P, z[lm_index, 0:2]) # associate to a known landmark
			nLM = calc_n_LM(X) # number of landmarks we currently know about
			if minid == nLM: # Landmark is a NEW landmark
				# Extend state and covariance matrix
				XAug = np.vstack((X, calc_LM_Pos(X, z[lm_index, :])))
				PAug = np.vstack((np.hstack((P, np.zeros((len(X), LM_STATE_DIMS)))),
								  np.hstack((np.zeros((LM_STATE_DIMS, len(X))), np.eye(LM_STATE_DIMS)))))
				X = XAug
				P = PAug
			lm = get_LM_Pos_from_state(X, minid)
			y, S, H = calc_innovation(lm, X, P, z[lm_index, 0:2], minid)

			K = (P @ H.T) @ np.linalg.inv(S) # Calculate Kalman Gain
			X = X + (K @ y)
			P = (np.eye(len(X)) - (K @ H)) @ P
	
		X[2] = pi_2_pi(X[2])
		return X, P