import math
import numpy as np
import scipy as sp
from scipy import linalg

import forward_kinematics as fk

def calc_inv_pos(angles, target_pos, target_ori, epsilon, right=True):
    p  = np.array([0,0,0,1])
    angs = np.array([a for a in angles])
    sum_old = 100000
    while True:
        pos, ori, j = fk.calc_fk_and_jacob(angs, jacob=True, right=right)
        J = _calc_invJ(j)
        delta_pos = np.matrix((target_pos-pos)[0:3]).transpose()
        delta_ori = np.matrix((target_ori-ori)[0:3]).transpose()
        
        delta = np.vstack((delta_pos, delta_ori))

        v_w = (J * delta).transpose()
        angs = np.squeeze(np.asarray(v_w)) + angs
        
        sum = 0
        
        for d in delta:
            sum = sum + math.fabs(d)
            
        #sum = np.sum(delta_pos)
        if sum < epsilon:
            break
            
        if sum > sum_old:
            print '# set_position error : Distance can not converged.'
            return None
        sum_old = sum
    return angs

def _calc_invJ(J, epsilon = 0.01):
    u, sigma, v = np.linalg.svd(J, full_matrices=True)
    sigma_ = [1/s if s > epsilon else 0 for s in sigma]
    rank_v = np.shape(J)[0]
    rank_h = np.shape(J)[1]

    return np.matrix(v.transpose()) * np.matrix(linalg.diagsvd(sigma_, rank_h, rank_v)) * np.matrix(u.transpose())

