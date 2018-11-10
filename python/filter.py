import numpy as np
import scipy.signal as ss

def Kalman_Filter(data, Q, R, x0, P0):
    num = len(data)
    K = np.zeros(num)
    X = np.zeros(num)
    P = np.zeros(num)

    X[0] = x0
    P[0] = P0

    for i in range(1, num):
        K[i] = P[i-1] / (P[i-1] + R)
        X[i] = X[i-1] + K[i] * (data[i] - X[i-1])
        P[i] = P[i-1] - K[i] * P[i-1] + Q

    return X

def Mean_Filter(data, windowSize):
    return ss.lfilter(np.ones(windowSize) / windowSize, 1, data)

