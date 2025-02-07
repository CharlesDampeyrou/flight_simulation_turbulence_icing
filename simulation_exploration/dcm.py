import numpy as np


def dcm(phi, theta, psi):
    """
    Direction Cosine Matrix (DCM) from Euler angles (phi, theta, psi)
    """
    sinR = np.sin(phi)
    cosR = np.cos(phi)
    sinP = np.sin(theta)
    cosP = np.cos(theta)
    sinY = np.sin(psi)
    cosY = np.cos(psi)
    return np.array(
        [
            [cosP * cosY, cosP * sinY, -sinP],
            [
                sinR * sinP * cosY - cosR * sinY,
                sinR * sinP * sinY + cosR * cosY,
                sinR * cosP,
            ],
            [
                cosR * sinP * cosY + sinR * sinY,
                cosR * sinP * sinY - sinR * cosY,
                cosR * cosP,
            ],
        ]
    )
