"""
Alonso Vega
November 7, 2020
Environment Class
"""
import numpy as np
from scipy import integrate


class Robot:
    __slots__ = '_x_initial', '_z_ref', '_q_ref',\
                '_K_matrix', '_Q_matrix', '_P_matrix',\
                '_k_0'

    def __init__(self, x_0, z_ref, q_ref):
        self._x_initial = x_0
        self._z_ref = z_ref
        self._q_ref = q_ref
        self._K_matrix = 0

    def f_function(self, x):
        return 1

    def M_matrix(self, m):
        m_1 = m[0]
        m_2 = m[1]
        m_3 = m[2]

        Mtilda_1 = np.array([np.cos(m_2 - m_3), -m_1*np.sin(m_2 - m_3), 0])
        Mtilda_2 = np.array([np.sin(m_2 - m_3), m_1*np.cos(m_2 - m_3), 1])

        M = np.array([Mtilda_1, Mtilda_2])
        return M

    def MDot_matrix(self, m, mDot):
        m_1 = m[0]
        m_2 = m[1]
        m_3 = m[2]
        mDot_1 = mDot[0]
        mDot_2 = mDot[1]
        mDot_3 = mDot[2]

        m_2_3    = m_2-m_3
        mDot_2_3 = mDot_2 - mDot_3
        trig_1 = np.array([-np.sin(m_2_3), np.cos(m_2_3)])
        trig_2 = np.array([-trig_1[0], trig_1[1]])

        M = np.array([mDot_2_3*trig_1, mDot_1*trig_1 - m_1*mDot_2_3*trig_2, [0, 0]])
        return M.transpose()

    def s_tilda(self):
        return 1

    def saturation(self):
        return 1

    def saturationDot(self):
        return 1

    def sliding_surface(self):
        return 1
