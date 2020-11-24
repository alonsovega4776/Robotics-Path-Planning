"""
Alonso Vega
November 23, 2020
Useful functions
"""
import numpy as np


def polar2xy(q):            # make sure angles are in radian
    ρ = q[0]
    φ = q[1]

    x = ρ * np.cos(φ)
    y = ρ * np.sin(φ)

    r = np.array([x, y])
    return r


def polar2xy_large(q_mat):  # make sure angles are in radian
    ρ_vect = q_mat[:, 0]
    φ_vect = q_mat[:, 1]

    x_c = np.multiply(ρ_vect, np.cos(φ_vect))
    y_c = np.multiply(ρ_vect, np.sin(φ_vect))

    return x_c, y_c


def SC(θ):
    return np.array([np.sin(θ),
                     np.cos(θ)])


def CS(θ):
    return np.array([np.cos(θ),
                     np.sin(θ)])

