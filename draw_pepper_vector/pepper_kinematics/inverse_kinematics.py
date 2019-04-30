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
        pos, ori, jv, jw = fk.calc_fk_and_jacob(angs, jacob=True, right=right)
        Jv = _calc_invJ(jv)
        delta_pos = np.matrix((target_pos-pos)[0:3]).transpose()
        v = (Jv * delta_pos).transpose()
        angs = np.squeeze(np.asarray(v)) + angs
        
        sum = 0
        for d in delta_pos:
            sum = sum + math.fabs(d)
        #sum = np.sum(delta_pos)
        if sum < epsilon:
            break
        if sum > sum_old:
            print '# set_position error : Distance can not converged.'
            return None
        sum_old = sum
        
    return angs

def calc_inv_ori(angles, target_pos, target_ori, epsilon, right=True):
    p  = np.array([0,0,0,1])
    angs = np.array([a for a in angles])
    sum_old = 100000
    while True:
        pos, ori, jv, jw = fk.calc_fk_and_jacob(angs, jacob=True, right=right)
        Jw = _calc_invJ(jw)
        delta_ori = np.matrix((target_ori-ori)[0:3]).transpose()
        
        w = (Jw * delta_ori).transpose()
        angs = np.squeeze(np.asarray(w)) + angs
        
        sum = 0
        for d in delta_ori:
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
    u, sigma, v = np.linalg.svd(J, full_matrices=0)
    sigma_ = [1/s if s > epsilon else 0 for s in sigma]
    rank = np.shape(J)[0]
    return np.matrix(v.transpose()) * np.matrix(linalg.diagsvd(sigma_, rank, rank)) * np.matrix(u.transpose())


