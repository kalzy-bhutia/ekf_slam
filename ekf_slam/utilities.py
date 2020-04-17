"""
author: Sanjay Thakur (@WanabeMarkovian)
blog: sanjaykthakur.com
"""

from ekf_slam.settings import *
from ekf_slam.models import motion_model
import math

def observation(xTrue, xd, u):
    """
    :param xTrue: the true pose of the system
    :param xd:    the current noisy estimate of the system
    :param u:     the current control input
    
    :returns:     Computes the true position, observations, dead reckoning (noisy) position, 
                  and noisy control function
    """
    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    z = np.zeros((0, 3))

    # The following lines of code use *XTrue* to calculate landmark distances and add noise to it explicitly.
    # Note that, in a real robot, this is not realistic. In that case, we will get a noisy measurement 
    # values directly from the sensors.
    for i in range(len(RFID[:, 0])): # Test all beacons, only add the ones we can see (within MAX_RANGE)

        dx = RFID[i, 0] - xTrue[0, 0]
        dy = RFID[i, 1] - xTrue[1, 0]
        d = math.sqrt(dx**2 + dy**2)
        angle = pi_2_pi(math.atan2(dy, dx) - xTrue[2, 0])
        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Qsim[0, 0]  # add noise
            anglen = angle + np.random.randn() * Qsim[1, 1]  # add noise
            zi = np.array([dn, anglen, i])
            z = np.vstack((z, zi))

    # add noise to input
    ud = np.array([[
        u[0, 0] + np.random.randn() * Rsim[0, 0],
        u[1, 0] + np.random.randn() * Rsim[1, 1]]]).T

    xd = motion_model(xd, ud)
    return xTrue, z, xd, ud


def calc_LM_Pos(x, z):
    """
    Calculates the pose in the world coordinate frame of a landmark at the given measurement. 

    :param x: [x; y; theta]
    :param z: [range; bearing]
    :returns: [x; y] for given measurement
    """
    zp = np.zeros((2, 1))

    zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
    zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])
    #zp[0, 0] = x[0, 0] + z[0, 0] * math.cos(x[2, 0] + z[0, 1])
    #zp[1, 0] = x[1, 0] + z[0, 0] * math.sin(x[2, 0] + z[0, 1])

    return zp


def get_LM_Pos_from_state(x, ind):
    """
    Returns the position of a given landmark
    
    :param x:   The state containing all landmark positions
    :param ind: landmark id
    :returns:   The position of the landmark
    """
    lm = x[ROBOT_STATE_DIMS + LM_STATE_DIMS * ind: ROBOT_STATE_DIMS + LM_STATE_DIMS * (ind + 1), :]

    return lm


def search_correspond_LM_ID(xAug, PAug, zi):
    """
    Landmark association with Mahalanobis distance.
    
    If this landmark is at least NEW_LM_THRESHOLD units away from all known landmarks, 
    it is a NEW landmark.
    
    :param xAug: The estimated state
    :param PAug: The estimated covariance
    :param zi:   the read measurements of specific landmark
    :returns:    landmark id
    """

    nLM = calc_n_LM(xAug)

    mdist = []

    for i in range(nLM):
        lm = get_LM_Pos_from_state(xAug, i)
        y, S, H = calc_innovation(lm, xAug, PAug, zi, i)
        mdist.append(y.T @ np.linalg.inv(S) @ y)

    mdist.append(NEW_LM_THRESHOLD)  # new landmark

    minid = mdist.index(min(mdist))

    return minid


def calc_input():
    v = 3.0  # [m/s]
    yawrate = 0.35  # [rad/s]
    u = np.array([[v, yawrate]]).T
    return u


def calc_innovation(lm, xEst, PEst, z, LMid):
    delta = lm - xEst[0:2]
    q = (delta.T @ delta)[0, 0]
    z_angle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
    zp = np.array([[math.sqrt(q), pi_2_pi(z_angle)]])
    y = (z - zp).T
    y[1] = pi_2_pi(y[1])
    H = jacob_h(q, delta, xEst, LMid + 1)
    S = H @ PEst @ H.T + Cx[0:2, 0:2]

    return y, S, H


def calc_n_LM(x):
    """
    Calculates the number of landmarks currently tracked in the state
    :param x: the state
    :returns: the number of landmarks n
    """
    n = int((len(x) - ROBOT_STATE_DIMS) / LM_STATE_DIMS)
    return n


def jacob_h(q, delta, x, i):
    sq = math.sqrt(q)
    G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                  [delta[1, 0], - delta[0, 0], - q, - delta[1, 0], delta[0, 0]]])

    G = G / q
    nLM = calc_n_LM(x)
    F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
    F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                    np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))

    F = np.vstack((F1, F2))
    H = G @ F
    return H


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def calc_n_LM(X):
    """
    Calculates the number of landmarks currently tracked in the state
    :param X: the state
    :returns: the number of landmarks n
    """
    n = int((len(X) - ROBOT_STATE_DIMS) / LM_STATE_DIMS)
    return n


def jacob_motion(x, u):
    """
    Calculates the jacobian of motion model. 
    
    :param x: The state, including the estimated position of the system
    :param u: The control function
    :returns: G:  Jacobian
              Fx: STATE_SIZE x (STATE_SIZE + 2 * num_landmarks) matrix where the left side is an identity matrix
    """
    
    # [eye(3) [0 x y; 0 x y; 0 x y]]
    Fx = np.hstack((np.eye(ROBOT_STATE_DIMS), np.zeros(
        (ROBOT_STATE_DIMS, LM_STATE_DIMS * calc_n_LM(x)))))

    jF = np.array([[0.0, 0.0, -DT * u[0] * math.sin(x[2, 0])],
                   [0.0, 0.0, DT * u[0] * math.cos(x[2, 0])],
                   [0.0, 0.0, 0.0]])

    G = np.eye(ROBOT_STATE_DIMS) + Fx.T @ jF @ Fx
    if calc_n_LM(x) > 0:
        print(Fx.shape)
    return G, Fx