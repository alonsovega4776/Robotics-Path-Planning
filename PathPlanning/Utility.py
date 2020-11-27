"""
November 23, 2020
Useful functions
"""
import numpy as np
import quaternion
import numba


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


def xy2polar_large(x_vect, y_vect):
    r_mat = np.stack([x_vect, y_vect])
    r_mat = np.transpose(r_mat)

    ρ_vect = np.linalg.norm(r_mat, axis=1)
    φ_vect = np.arctan2(y_vect, x_vect)

    return ρ_vect, φ_vect


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


def metric(q_a, q_b, metric_weight):         # make sure angles are in radian
    r_a = polar2xy(q_a)
    r_b = polar2xy(q_b)
    Δr  = r_b - r_a

    metric_x = np.abs(Δr[0])
    metric_x = metric_x*metric_x

    metric_y = np.abs(Δr[1])
    metric_y = metric_y*metric_y

    θ_a_half = q_a[2]/2
    θ_b_half = q_b[2]/2

    h_a      = np.quaternion(np.cos(θ_a_half), 0, 0, np.sin(θ_a_half))
    h_b      = np.quaternion(np.cos(θ_b_half), 0, 0, np.sin(θ_b_half))
    dot_prod = np.dot(quaternion.as_float_array(h_a), quaternion.as_float_array(h_b))
    metric_θ = np.arccos(np.abs(dot_prod))
    metric_θ = metric_θ * metric_θ

    metric = np.dot(metric_weight, np.array([metric_x, metric_y, metric_θ]))
    metric = np.sqrt(metric)
    return metric




