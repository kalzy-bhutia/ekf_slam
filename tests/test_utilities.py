import pytest
import numpy as np
import math
from ekf_slam import utilities as utils


@pytest.mark.parametrize("x, z, lm_pos", [(np.array([[1], [2], [math.pi/4]]), np.array([[2], [math.pi/2]]), np.array([[-0.41421356], [3.41421356]])), ])
def test_calc_LM_Pos(x, z, lm_pos):
    est_lm_pos = utils.calc_LM_Pos(x, z)
    assert round(est_lm_pos[0, 0], 2) == round(lm_pos[0, 0], 2), "x axes don't match"
    assert round(est_lm_pos[1, 0], 2) == round(lm_pos[1, 0], 2), "y axes don't match"