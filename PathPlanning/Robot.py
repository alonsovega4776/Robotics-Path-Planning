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
                '_q_ε',\
                '_γ_1', '_γ_2'

    def __init__(self, x_initial, q_ref, t_1, t_2, step_number):
        self._x_0 = x_initial
        self._q_ref = q_ref
        self._q_refDot = [0, 0, 0]

        self._K_matrix = np.array([[4, 0, 0], [0, 4, 0], [0, 0, 0.02]])
        self._Q_matrix = np.array([[5, 0.01], [0.002, 3.5]])
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

        self._γ_1 = 10.0
        self._γ_2 = 0.55

    def get_trajectory(self):
        sol = integrate.odeint(self.f_function, self._x_0, self._t)

        fig, axes = plt.subplots(nrows=2, ncols=3, sharex=False)

        ρ_ref = np.ones_like(self._t)*self._q_ref[0]
        φ_ref = np.ones_like(self._t)*np.degrees(self._q_ref[1])
        θ_ref = np.ones_like(self._t)*np.degrees(self._q_ref[2])

        font = {'family': 'serif',
                'color': 'darkred',
                'weight': 'normal',
                'size': 10,
                }

        axes[0, 0].plot(self._t, sol[:, 0], 'b')
        axes[0, 0].plot(self._t, ρ_ref, 'g--')
        axes[0, 0].set_title('ρ$_c$(t): Radial Distance', font)
        axes[0, 0].set_xlabel('t [s]')
        axes[0, 0].set_ylabel('[m]')
        axes[0, 0].grid(True)
        axes[0, 1].plot(self._t, np.degrees(sol[:, 1]), 'b')
        axes[0, 1].plot(self._t, φ_ref, 'g--')
        axes[0, 1].set_title('φ$_c$(t): Angular Coordinate', font)
        axes[0, 1].set_xlabel('t [s]')
        axes[0, 1].set_ylabel('[°]')
        axes[0, 1].grid(True)
        axes[0, 2].plot(self._t, np.degrees(sol[:, 2]), 'b')
        axes[0, 2].plot(self._t, θ_ref, 'g--')
        axes[0, 2].set_title('θ$_c$(t): Orientation', font)
        axes[0, 2].set_xlabel('t [s]')
        axes[0, 2].set_ylabel('[°]')
        axes[0, 2].grid(True)

        axes[1, 0].plot(self._t, np.abs(sol[:, 0] - ρ_ref), 'r')
        axes[1, 0].plot(self._t, np.zeros_like(self._t), 'g--')
        axes[1, 0].set_title('ρ$_e$(t): Error', font)
        axes[1, 0].set_xlabel('t [s]')
        axes[1, 0].set_ylabel('[m]')
        axes[1, 0].grid(True)
        axes[1, 1].plot(self._t, np.abs(np.degrees(sol[:, 1]) - φ_ref), 'r')
        axes[1, 1].plot(self._t, np.zeros_like(self._t), 'g--')
        axes[1, 1].set_title('φ$_e$(t): Error', font)
        axes[1, 1].set_xlabel('t [s]')
        axes[1, 1].set_ylabel('[°]')
        axes[1, 1].grid(True)
        axes[1, 2].plot(self._t, np.abs(np.degrees(sol[:, 2]) - θ_ref), 'r')
        axes[1, 2].plot(self._t, np.zeros_like(self._t), 'g--')
        axes[1, 2].set_title('θ$_e$(t): Error', font)
        axes[1, 2].set_xlabel('t [s]')
        axes[1, 2].grid(True)

        fig.show()

        return sol

    # _______________________________________xDot = f(x,t,u______)______________________________________________________
    def f_function(self, x, t):
        q_c = x[0:3]
        z_c   = x[3:5]

        if q_c[0] == 0:
            q_c[0] = self._q_ε

        q_ref    = self._q_ref          # reference
        q_refDot = self._q_refDot       # reference derivative
        q_e      = q_c - q_ref          # error in q_c

        f = self._f                     # disturbance

        S_null = self.null_matrix(q_c)  # null matrix
        q_cDot = np.matmul(S_null, z_c)   # x[0:3]Dot = f(x)

        q_eDot = q_cDot - q_refDot      # error in q_cDot

        # u = self.controller_chwa(q_c, q_cDot, q_e, q_eDot)
        u = self.controller_yang(q_c, z_c, q_cDot, q_e, q_eDot)

        zDot  = u - f                   # # x[0:3]Dot = f(x)
        return np.concatenate([q_cDot, zDot])
    # _______________________________________xDot = f(x,t,u______)______________________________________________________

    # _____________________________________________________Control______________________________________________________
    def controller_yang(self, q_c, z_c, q_cDot, q_e, q_eDot,):
        ρ_c = q_c[0]
        φ_c = q_c[1]
        θ_c = q_c[2]

        q_ref    = self._q_ref              # reference
        M_ref    = self.M_matrix(q_ref)
        q_refDot = self._q_refDot
        z_ref    = np.matmul(M_ref, q_refDot)
        q_e      = q_c - q_ref              # error in q_c

        ρ_e = q_e[0]
        φ_e = q_e[1]
        θ_e = q_e[2]

        ρ_eDot = q_eDot[0]
        φ_eDot = q_eDot[1]
        θ_eDot = q_eDot[2]

        # _____________________________Control Parameters_______________________________________________________________
        Q   = self._Q_matrix
        P   = self._P_matrix
        γ_1 = self._γ_1
        γ_2 = self._γ_2
        Γ   = np.diag([γ_1, γ_2])
        # _____________________________Control Parameters_______________________________________________________________

        s = self.sliding_surface_yang(q_c, q_e, q_eDot)   # sliding surface

        angle_diff = φ_c - θ_c
        if angle_diff == 0:
            print('\nERROR: divided by 0 in controller_yang\n')
            quit()
        U = np.array([[1/np.cos(angle_diff), 0],
                      [0                   , 1]])

        u_1 = -np.matmul(Q, s)

        u_2 = -np.matmul(P, self.saturation(s))

        u_3 = -np.matmul(Γ, np.array([ρ_eDot, θ_eDot]))

        u_41 = self.R_function(q_ref, z_ref) - self.R_function(q_c, z_c)
        u_42 = -self.saturation(θ_e)*self.saturation(φ_e)*φ_eDot
        u_4 = np.array([u_41,
                        u_42])

        u = u_1 + u_2 + u_3 + u_4
        u = np.matmul(U, u)
        return u

    def controller_chwa(self, q_c, q_cDot, q_e, q_eDot):
        ρ_c = q_c[0]
        φ_c = q_c[1]
        θ_c = q_c[2]

        ρ_e = q_e[0]
        φ_e = q_e[1]
        θ_e = q_e[2]

        φ_cDot = q_cDot[1]
        θ_cDot = q_cDot[2]

        ρ_eDot = q_eDot[0]
        φ_eDot = q_eDot[1]

        # _____________________________Control Parameters_______________________________________________________________
        K = self._K_matrix
        Q = self._Q_matrix
        P = self._P_matrix
        k_0 = self._k_0
        # _____________________________Control Parameters_______________________________________________________________

        # _______________________________________________________________________Miscellaneous__________________________
        M_c = self.M_matrix(q_c)
        M_cDot = self.MDot_matrix(q_c, q_cDot)

        M_c_2x2 = self.M_matrix(np.array([1, φ_c, θ_c]))
        M_c_2x2 = M_c_2x2[:, 0:2]
        M_c_2x2Dot = self.MDot_matrix(np.array([1, φ_c, θ_c]), np.array([0, φ_cDot, θ_cDot]))
        M_c_2x2Dot = M_c_2x2Dot[:, 0:2]

        s_inner = self.sliding_surface_inner_chwa(q_e, q_eDot)
        s_θ = s_inner[2]
        s_θDot = 0
        s = self.sliding_surface_chwa(q_c, q_e, q_eDot)         # sliding surface
        # _______________________________________________________________________Miscellaneous__________________________

        u_11 = np.matmul(K, q_e)
        u_1 = -np.matmul(M_cDot, u_11)

        u_21 = np.matmul(K, q_eDot)
        u_2 = -np.matmul(M_c, u_21)

        u_31 = np.matmul(M_c_2x2Dot, self.saturation([ρ_e, φ_e])) * np.abs(s_θ)
        u_32_1 = self.saturationDot([ρ_e, φ_e], [ρ_eDot, φ_eDot]) * np.abs(s_θ)
        u_32_2 = self.saturation([ρ_e, φ_e]) * np.sign(s_θ) * s_θDot
        u_32 = np.matmul(M_c_2x2, u_32_1 + u_32_2)
        u_3 = -k_0 * (u_31 + u_32)

        u_4 = -np.matmul(Q, s)

        u_5 = -np.matmul(P, np.sign(s))

        u = u_1 + u_2 + u_3 + u_4 + u_5
        return u
    # _____________________________________________________Control______________________________________________________

    def M_matrix(self, m):
        m_1   = m[0]
        m_2   = m[1]
        m_3   = m[2]
        m_2_3 = m_2 - m_3

        Mtilda_1 = np.array([np.cos(m_2_3), -m_1*np.sin(m_2_3), 0])
        Mtilda_2 = np.array([np.sin(m_2_3),  m_1*np.cos(m_2_3), 1])

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
        m_2_3Dot = mDot_2 - mDot_3
        trig_1 = np.array([-np.sin(m_2_3), np.cos(m_2_3)])
        trig_2 = np.array([-trig_1[1], trig_1[0]])

        M = np.array([m_2_3Dot*trig_1, mDot_1*trig_1 + m_1*m_2_3Dot*trig_2, [0, 0]])
        return M.transpose()

    def saturation(self, error, φ=1):
        return np.clip(error, -φ, φ)

    def saturationDot(self, error, errorDot, φ=1):
        satDot = []
        for i in range(0,len(error)-1):
            if np.abs(error[i]) > φ:
                satDot.append(0)
            else:
                satDot.append((1/φ)*errorDot[i])
        return np.array(satDot)

    def sliding_surface_inner_chwa(self, q_e, q_eDot):
        return q_eDot + np.matmul(self._K_matrix, q_e)

    def sliding_surface_chwa(self, q_c, q_e, q_eDot):
        s_tilda_1 = self.sliding_surface_inner_chwa(q_e, q_eDot)
        s_theta   = s_tilda_1[2]

        ρ_c   = q_c[0]
        if ρ_c == 0:
            ρ_c = self._q_ε

        q_e_2 = np.concatenate([q_e[0:2], [1]])
        s_tilda_21 = self._k_0 * np.abs(s_theta) * self.saturation(q_e_2)
        s_tilda_22 = np.array([1, 1/ρ_c, s_theta])
        s_tilda_2  = np.multiply(s_tilda_21, s_tilda_22)

        M_c = self.M_matrix(q_c)
        s = np.matmul(M_c, s_tilda_1 + s_tilda_2)
        return s

    def sliding_surface_yang(self, q_c, q_e, q_eDot):
        γ_1 = self._γ_1
        γ_2 = self._γ_2

        ρ_e = q_e[0]
        φ_e = q_e[1]
        θ_e = q_e[2]

        ρ_eDot = q_eDot[0]
        θ_eDot = q_eDot[2]

        s_1 = ρ_eDot + γ_1*ρ_e
        s_2 = θ_eDot + γ_2*θ_e + self.saturation(θ_e)*np.abs(φ_e)
        s   = np.array([s_1, s_2])
        return s

    def R_function(self, q, z):
        ρ = q[0]
        φ = q[1]
        θ = q[2]

        if ρ == 0:
            ρ = self._q_ε

        v = z[0]
        ω = z[1]

        angle_diff = φ - θ
        sin_diff = np.sin(angle_diff)

        return ((v*v)/ρ)*(sin_diff*sin_diff) + v*ω*sin_diff

    def null_matrix(self, q_c):
        ρ_c = q_c[0]
        φ_c = q_c[1]
        θ_c = q_c[2]

        if ρ_c == 0:
            ρ_c = self._q_ε

        S_1 = np.array([np.cos(φ_c - θ_c), -(1/ρ_c)*np.sin(φ_c - θ_c), 0])
        S_2 = np.array([0, 0, 1])

        S = np.column_stack([S_1, S_2])
        return S

