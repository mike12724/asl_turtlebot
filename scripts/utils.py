import numpy as np

THETA_EPS = np.pi / 4.0
POS_EPS = 0.1

def wrapToPi(a):
    if isinstance(a, list):
        return [(x + np.pi) % (2*np.pi) - np.pi for x in a]
    return (a + np.pi) % (2*np.pi) - np.pi
