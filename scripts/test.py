import numpy as np
import scipy.linalg as linalg
import matplotlib.pyplot as plt
# from GetRadar import GetRadar
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.common import Q_discrete_white_noise

A = []
A.insert(0,3)
A.insert(0,4)
print(A)

A.pop()
print(A)
