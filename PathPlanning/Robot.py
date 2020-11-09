"""
Alonso Vega
November 7, 2020
Environment Class
"""
import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt
from scipy.misc import derivative


class Robot:
    __slots__ = '_x_0', '_q_ref', '_q_refDot',\
                '_K_matrix', '_Q_matrix', '_P_matrix',\
                '_k_0', '_f',\
                '_N', '_t_1', '_t_2', '_t',\
                '_q_ε'

    def __init__(self, x_initial, q_ref, t_1, t_2, step_number):
        self._x_0 = x_initial
        self._q_ref = q_ref
        self._q_refDot = [0, 0, 0]

        self._K_matrix = np.array([[2, 0, 0], [0, 2, 0], [0, 0, 0.002]])
        self._Q_matrix = np.array([[10, 0], [0, 10]])
        self._f = np.array([0.5, 0.5])
        F = np.diag(self._f)
        η = np.diag([0.5, 0.5])
        self._P_matrix = η + F
        self._k_0 = 1.1
        self._t_1 = t_1
        self._t_2 = t_2
        self._N = step_number
        self._t = t = np.linspace(t_1, t_2, step_number)
        self._q_ε = 0.01

    def get_trajectory(self):
        sol = integrate.odeint(self.f_function, self._x_0, self._t)

        plt.plot(self._t, sol[:, 0], 'b')
        plt.xlabel('t')
        plt.grid()
        plt.show()

        return sol

    def f_function(self, x, t):
        q_c = x[0:3]
        z   = x[3:5]

        if q_c[0] == 0:
            q_c[0] = self._q_ε

        q_ref    = self._q_ref
        q_refDot = self._q_refDot
        K        = self._K_matrix
        Q        = self._Q_matrix
        P        = self._P_matrix
        k_0      = self._k_0
        f        = self._f

        q_e   = q_c - q_ref

        ρ_e = q_e[0]
        φ_e = q_e[1]
        θ_e = q_e[2]

        ρ_c = q_c[0]
        φ_c = q_c[1]
        θ_c = q_c[2]
        v_c = z[0]
        ω_c = z[1]

        q_cDot   = np.matmul(self.null_matrix(q_c), z)
        φ_cDot = q_cDot[1]
        θ_cDot = q_cDot[2]

        q_eDot   = q_cDot - q_refDot
        ρ_eDot = q_eDot[0]
        φ_eDot = q_eDot[1]

        M_cDot = self.MDot_matrix(q_c, q_cDot)
        u_11 = np.matmul(K, q_e)
        u_1  = -np.matmul(M_cDot, u_11)

        M_c = self.M_matrix(q_c)
        u_21 = np.matmul(K, q_eDot)
        u_2  = -np.matmul(M_c, u_21)

        M_c_2x2    = self.M_matrix(np.array([1, φ_c, θ_c]))
        M_c_2x2 = M_c_2x2[:, 0:2]
        M_c_2x2Dot = self.MDot_matrix(np.array([1, φ_c, θ_c]),
                                      np.array([0, φ_cDot, θ_cDot]))
        M_c_2x2Dot = M_c_2x2Dot[:, 0:2]
        s_hat = self.sliding_surface_inner(q_e, q_eDot)
        s_θ = s_hat[2]
        s_θDot = 0.1
        u_31 = np.matmul(M_c_2x2Dot, self.saturation([ρ_e, φ_e]))*np.abs(s_θ)
        u_32_1 = self.saturationDot([ρ_e, φ_e], [ρ_eDot, φ_eDot])*np.abs(s_θ)
        u_32_2 = self.saturation([ρ_e, φ_e])*np.sign(s_θ)*s_θDot
        u_32 = np.matmul(M_c_2x2, u_32_1 + u_32_2)
        u_3 = -k_0*(u_31 + u_32)

        s = self.sliding_surface(q_c, q_e, q_eDot)
        u_4 = -np.matmul(Q, s)

        u_5 = -np.matmul(P, np.sign(s))

        u = u_1 + u_2 + u_3 + u_4 + u_5
        zDot  = u - f

        return np.concatenate([q_cDot, zDot])

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

        m_2_3    = m_2    - m_3
        mDot_2_3 = mDot_2 - mDot_3
        trig_1 = np.array([-np.sin(m_2_3), np.cos(m_2_3)])
        trig_2 = np.array([-trig_1[0], trig_1[1]])

        M = np.array([mDot_2_3*trig_1, mDot_1*trig_1 - m_1*mDot_2_3*trig_2, [0, 0]])
        return M.transpose()

    def saturation(self, error, φ=1):
        return -np.clip(error, -φ, φ)

    def saturationDot(self, error, errorDot, φ=1):
        satDot = []
        for i in range(0,len(error)-1):
            if np.abs(error[i]) > φ:
                satDot.append(0)
            else:
                satDot.append(-(1/φ)*errorDot[i])
        return np.array(satDot)

    def sliding_surface_inner(self, q_e, q_eDot):
        return q_eDot + np.matmul(self._K_matrix, q_e)

    def sliding_surface(self, q_c, q_e, q_eDot):
        s_tilda_1 = self.sliding_surface_inner(q_e, q_eDot)
        s_theta = s_tilda_1[2]

        q_e_2 = np.concatenate([q_e[0:2], [-1]])
        s_tilda_21 = self._k_0 * np.abs(s_theta) * self.saturation(q_e_2)

        ρ_c = q_c[0]
        s_tilda_22 = np.array([1, 1/ρ_c, s_theta])
        s_tilda_2 = np.multiply(s_tilda_21, s_tilda_22)

        M = self.M_matrix(q_c)
        s = np.matmul(M, s_tilda_1 + s_tilda_2)
        return s

    def null_matrix(self, q_c):
        ρ_c = q_c[0]
        φ_c = q_c[1]
        θ_c = q_c[2]

        S_1 = np.array([np.cos(φ_c - θ_c), (1/ρ_c)*np.sin(φ_c - θ_c), 0])
        S_2 = np.array([0, 0, 1])

        S = np.column_stack([S_1, S_2])
        return S

