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


def xy2polar(x, y):
    r = np.array([x, y])

    ρ = np.linalg.norm(r)
    φ = np.arctan2(y, x)

    return ρ, φ


def polar2xy_large(q_mat):  # make sure angles are in radian
    ρ_vect = q_mat[:, 0]
    φ_vect = q_mat[:, 1]

    x_c = np.multiply(ρ_vect, np.cos(φ_vect))
    y_c = np.multiply(ρ_vect, np.sin(φ_vect))

    return x_c, y_c


def SC_vect(θ):
    return np.array([np.sin(θ),
                     np.cos(θ)])


def CS_vect(θ):
    return np.array([np.cos(θ),
                     np.sin(θ)])


def heading_direction(q_1, q_2, degrees=False):
    (x_1, y_1) = polar2xy(q_1)
    (x_2, y_2) = polar2xy(q_2)

    Δr     = np.array([x_2 - x_1,
                       y_2 - y_1])
    Δr_hat = Δr/np.linalg.norm(Δr)
    e_1    = np.array([1, 0])

    θ_r = np.dot(Δr_hat, e_1)
    θ_r = np.arccos(θ_r)

    if Δr[1] < 0:
        θ_r = -θ_r

    if degrees:
        return np.degrees(θ_r)
    else:
        return θ_r


def spherical2xyz(q):
    ρ = q[0]
    φ = q[1]
    θ = q[2]

    (x, y) = polar2xy(q)
    z      = ρ*np.tan(θ)

    p = np.array([x, y, z])
    return p





