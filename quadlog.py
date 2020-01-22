import numpy as np
import quadrotor as quad

class quadlog:
    def __init__(self, time):
        self.d = np.zeros(time.size)
        self.pqr_h = np.zeros((time.size, 3))
        self.xyz_h = np.zeros((time.size, 3))
        self.v_ned_h = np.zeros((time.size, 3))
        self.w_h = np.zeros((time.size, 4))
        self.xi_g_h = np.zeros(time.size)
        self.xi_CD_h = np.zeros(time.size)
        self.b_h = np.zeros(time.size)
