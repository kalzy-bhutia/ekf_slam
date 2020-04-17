import numpy as np
import math
from ekf_slam import utilities as utils


# calc_LM_Pos, get_LM_Pos_from_state, search_correspond_LM_ID, calc_input, calc_innovation, calc_n_LM, jacob_h, pi_2_pi, calc_n_LM, jacob_motion

def test_calc_LM_Pos():
    x = np.array([[1], [2], [math.pi/4]])
    z = np.array([[2], [math.pi/2]])
    est_lm_pos = utils.calc_LM_Pos(x, z)
    lm_pos = np.array([[-0.41421356], [3.41421356]])
    assert round(est_lm_pos[0, 0], 2) == round(lm_pos[0, 0], 2), "x axes don't match"
    assert round(est_lm_pos[1, 0], 2) == round(lm_pos[1, 0], 2), "y axes don't match"

'''
def test_get_LM_Pos_from_state():
    x = 5
    y = 6
    assert x + 1 == y, "test failed"


def test_search_correspond_LM_ID():
    pass


def test_calc_input():
    pass


def test_calc_innovation():
    pass


def test_calc_n_LM():
    pass


def test_jacob_h():
    pass


def test_pi_2_pi():
    pass


def test_calc_n_LM():
    pass


def test_jacob_motion():
    pass
    
'''