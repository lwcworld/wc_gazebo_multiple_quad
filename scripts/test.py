import numpy as np
import scipy.linalg as linalg
import matplotlib.pyplot as plt
# from GetRadar import GetRadar
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.common import Q_discrete_white_noise

res_1 = [1,2]
Sig_1 = [[1,0],[0,1]]

a = np.dot(np.dot(res_1, np.linalg.inv(Sig_1)), res_1)
print(a)