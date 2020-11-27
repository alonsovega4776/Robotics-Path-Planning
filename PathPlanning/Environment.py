"""
Alonso Vega
October 29, 2020
Environment Class
"""
import Robot
import Tree
import Utility as util

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import collections as pltC
from matplotlib import patches
import Camera


class Environment:
    __slots__ = '_xMin', '_xMax', '_yMin', '_yMax',\
                '_obstacleList',\
                '_goal', '_start', \
                '_axes', '_figure', \
                '_robot', '_RRTtree', \
                '_cov_matrix', \
                '_kd_Tree', \
                '_xTilda', '_camera', \
                '_dt_head_min_pph', '_dt_head_max_pph', '_μ_tHeadControl_pph', '_Σ_tHeadControl_pph', \
                '_Δ_trajectory', '_odeIterGuassMax', '_odeIterMax', \
                '_headSD_Guass', \
                '_ε', '_metric_weight', \
                '_ε_collision', '_ε_goal', '_goal_indices'

    def __init__(self, X, Y, obstacle_list, initial_state, goal):
        self._xMin = X[0]
        self._xMax = X[1]

        self._yMin = Y[0]
        self._yMax = Y[1]

        self._obstacleList = obstacle_list

        self._robot = Robot.Robot(initial_state,
                                  q_ref=initial_state[0:3],
                                  t_1=0.0, t_2=2.0, step_number=100)

        self._RRTtree = Tree.Tree(initial_state)

        self._goal  = goal
        self._start = util.polar2xy(initial_state[0:2])

        self._cov_matrix = np.diag([1.5, 1.5])

        self._figure, self._axes = plt.subplots()
        self._figure.set_figheight(10.0)
        self._figure.set_figwidth(10.0)
        self._axes.grid(True)

        self._camera = Camera.Camera(self._figure)

        self._dt_head_min_pph    = 25  # pph
        self._dt_head_max_pph    = 50  # pph
        self._μ_tHeadControl_pph = np.array([10, 25])/100  # pph in decimal
        self._Σ_tHeadControl_pph = np.array([5, 10])/100   # pph in decimal

        self._xTilda = []
        self._Δ_trajectory = 5
        self._odeIterGuassMax = 4       # actual number of normal control calls is one less than this number
        self._odeIterMax      = 6

        self._headSD_Guass = 0.1

        self._ε = 1.0
        self._metric_weight = np.array([1.0, 1.0, 2.0])

        self._ε_collision = 0.1
        self._ε_goal      = 1.0
        self._goal_indices = []

    def get_camera(self):
        return self._camera

    def get_robot(self):
        return self._robot

    def x_max(self):
        return self._xMax

    def y_max(self):
        return self._yMax

    def x_min(self):
        return self._xMin

    def y_min(self):
        return self._yMin

    def goal(self):
        return self._goal

    def start(self):
        return self._start

    def set_goal(self, new_goal):
        self._goal = new_goal

    def set_start(self, new_initial_state):
        self._start = new_initial_state

    def obstacles(self):
        return self._obstacleList

    def print_environment(self):

        O_points = []
        limit = 1
        O_points.append([self._xMin, self._yMin])
        O_points.append([self._xMin - limit, self._yMin - limit])
        O_points.append([self._xMin - limit, self._yMax + limit])
        O_points.append([self._xMax + limit, self._yMax + limit])
        O_points.append([self._xMax, self._yMax])
        O_points.append([self._xMin, self._yMax])
        O_points.append([self._xMin, self._yMin])
        O_points = np.array(O_points)
        self._axes.fill(O_points[:, 0], O_points[:, 1], color=(0, 0, 0, 0.15))
        O_points = []
        O_points.append([self._xMin, self._yMin])
        O_points.append([self._xMin - limit, self._yMin - limit])
        O_points.append([self._xMax + limit, self._yMin - limit])
        O_points.append([self._xMax + limit, self._yMax + limit])
        O_points.append([self._xMax, self._yMax])
        O_points.append([self._xMax, self._yMin])
        O_points.append([self._xMin, self._yMin])
        O_points = np.array(O_points)
        self._axes.fill(O_points[:, 0], O_points[:, 1], color=(0, 0, 0, 0.15))

        points = []
        i_1 = 0
        for obstacle in self._obstacleList:
            O_points = []
            for v in obstacle.boundary_vertices():
                x = v.x_value()
                y = v.y_value()
                points.append([x, y])
                O_points.append([x, y])
                self._axes.annotate('v_' + f'{i_1}' + '' + f'{v.id()}', (x, y),
                              (0.65*x + 0.35*obstacle.centroid()[0], 0.65*y + 0.35*obstacle.centroid()[1]))
            O_points = np.array(O_points)
            self._axes.fill(O_points[:, 0], O_points[:, 1], color=(0, 0, 0, 0.15))
            i_1 = i_1 + 1
        points = np.array(points)
        self._axes.scatter(points[:, 0], points[:, 1], s=30.0, color=(0, 0, 1, 0.5))

        points = []
        i_1 = 0
        for obstacle in self._obstacleList:
            O_cent = obstacle.centroid()
            points.append(O_cent)
            self._axes.annotate('O_' + f'{i_1}', (O_cent[0], O_cent[1]))
            i_1 = i_1 + 1
        points = np.array(points)
        self._axes.scatter(points[:, 0], points[:, 1],
                     s=100.0, color=(0, 0, 0, 0.2), marker='h')

        points = []
        for obstacle in self._obstacleList:
            for e in obstacle.edges():
                (v_1, v_2) = e.end_vertices()
                points.append([(v_1.x_value(), v_1.y_value()), (v_2.x_value(), v_2.y_value())])
        points.append([(self._xMin, self._yMin), (self._xMin, self._yMax)])
        points.append([(self._xMin, self._yMax), (self._xMax, self._yMax)])
        points.append([(self._xMax, self._yMax), (self._xMax, self._yMin)])
        points.append([(self._xMax, self._yMin), (self._xMin, self._yMin)])

        segments = pltC.LineCollection(points, linewidths=0.75, colors=(0, 0, 1, 1))
        self._axes.add_collection(segments)

        goal_art = patches.Circle((self._goal[0], self._goal[1]),
                               self._ε_goal,
                               color=(1.0, 0.0, 0.0, 0.15))
        self._axes.add_artist(goal_art)


        self._axes.set_xlim(self._xMin - limit, self._xMax + limit)
        self._axes.set_ylim(self._yMin - limit, self._yMax + limit)

    def axes(self):
        return self._axes

    def figure(self):
        return self._figure

    def collision(self, point, plot=False):
        ε = self._ε_collision

        point = np.array(point)
        col_vect = (point < np.array([self._xMin, self._yMin])) \
                   | (np.isclose(point, [self._xMin, self._yMin], atol=ε, rtol=0.0))\
                   | (point > np.array([self._xMax, self._yMax]))\
                   | (np.isclose(point, [self._xMax, self._yMax], atol=ε, rtol=0.0))
        collision = (col_vect[0] | col_vect[1])
        if collision:
            if plot:
                self.paint_collision_point(point[0], point[1], collision)
            return True

        for obstacle in self._obstacleList:
            col_vect = np.matmul(obstacle.boundary()['N'], point) + obstacle.boundary()['b']
            col_vect = (col_vect < 0) | (np.isclose(col_vect, np.zeros(len(col_vect)), atol=ε, rtol=0.0))
            collision = (np.sum(col_vect.astype(int)) == len(col_vect))
            if collision:
                if plot:
                    self.paint_collision_point(point[0], point[1], collision)
                return True
        if plot:
            self.paint_collision_point(point[0], point[1], collision)
        return False

    def paint_collision_point(self, x, y, collision):
        color = []
        if collision:
            color.append((1.0, 0.0, 0.0, 0.5))
        else:
            color.append((0.4, 0.75, 0.1, 0.5))

        self._axes.scatter(x, y,
                           s=5.00, color=color, marker='o')

    def sample(self, n, distribution):
        if distribution == 'U':
            x_rand = np.random.uniform(self._xMin, self._xMax, n)
            y_rand = np.random.uniform(self._yMin, self._yMax, n)
        elif distribution == 'N':
            q_0 = self._robot.get_x_0()[0:3]
            r_0 = util.polar2xy(q_0)

            Σ = self._cov_matrix
            μ = np.array([r_0[0], r_0[1]])

            sample = np.random.multivariate_normal(μ, Σ, n)
            x_rand = sample[:, 0]
            y_rand = sample[:, 1]
        else:
            print('\nERROR: no such distribution.\n')
        return x_rand, y_rand

    # ________________________________________________Integration_______________________________________________________
    def draw_robot_trajectory(self, plot=False, draw_successful_trajectory=False):
        (self._xTilda, info) = self._robot.get_trajectory(degrees=False, plot=plot)

        odeIterGuassMax = self._odeIterGuassMax
        odeIterMax      = self._odeIterMax
        ode_iter = 1
        while info['message'] != 'Integration successful.':
            if ode_iter < odeIterGuassMax:
                (t_head_min, t_head_max) = self.set_random_time_control('N')
                print('\nChanging Control: Normal heading time control: (', t_head_min, ',', t_head_max, ')')
            else:
                if (ode_iter + 1) == odeIterMax:
                    (t_head_min, t_head_max) = self._robot.get_time_duration()
                    self._robot.set_t_head_max(t_head_max)
                    self._robot.set_t_head_min(t_head_min)
                    print('\nChanging Control: Full heading time control: (', t_head_min, ',', t_head_max, ')')
                else:
                    (t_head_min, t_head_max) = self.set_random_time_control('U')
                    print('\nChanging Control: Uniform heading time control: (', t_head_min, ',', t_head_max, ')')

            ode_iter = ode_iter + 1
            print('ERROR: integration failed, trying again. Iteration:', ode_iter, ' \n')
            (self._xTilda, info) = self._robot.get_trajectory(degrees=False, plot=plot)

            if ode_iter >= odeIterMax:
                if info['message'] != 'Integration successful.':
                    print('\nERROR: integration failed, max iterations reached. Total Iteration:', ode_iter, ' \n')
                    return info['message']
                else:
                    break
        print('\nIntegration successful after ', ode_iter, ' iterations. \n')

        if draw_successful_trajectory:
            r_cTilda = self._xTilda[:, 0:2]
            (x_c, y_c) = util.polar2xy_large(r_cTilda)

            self._axes.plot(x_c, y_c, color='black', linestyle='--', linewidth=0.5)
        return info['message']
    # ________________________________________________Integration_______________________________________________________

    def draw_good_trajectory(self):
        r_cTilda = self._xTilda[:, 0:2]
        (x_c, y_c) = util.polar2xy_large(r_cTilda)

        self._axes.plot(x_c, y_c)

    def play_robot_trajectory(self):
        xTilda = self._xTilda

        for i in range(0, len(xTilda[:, 1])):
            self.animate(i)

        anime = self._camera.animate()
        return anime

    def animate(self, i):
        q_cTilda   = self._xTilda[:, 0:3]
        (x_c, y_c) = util.polar2xy_large(q_cTilda)

        r_c = np.array([x_c[i], y_c[i]])
        θ_c = q_cTilda[i, 2]

        R       = self._robot.get_wheel_radius()
        R_robot = self._robot.get_base_radius()
        l       = self._robot.get_wheel_center_distance()

        wheel_rightFront = r_c + np.matmul(np.diag([1, -1]), l * util.SC_vect(θ_c)) \
                               + np.matmul(np.diag([1, 1]), R * util.CS_vect(θ_c))
        wheel_rightBack  = r_c + np.matmul(np.diag([1, -1]), l * util.SC_vect(θ_c)) \
                               + np.matmul(np.diag([-1, -1]), R * util.CS_vect(θ_c))
        wheel_leftFront  = r_c + np.matmul(np.diag([-1, 1]), l * util.SC_vect(θ_c)) \
                               + np.matmul(np.diag([1, 1]), R * util.CS_vect(θ_c))
        wheel_leftBack   = r_c + np.matmul(np.diag([-1, 1]), l * util.SC_vect(θ_c)) \
                               + np.matmul(np.diag([-1, -1]), R * util.CS_vect(θ_c))

        head_right  = r_c + np.matmul(np.diag([1, -1]), l * util.SC_vect(θ_c))
        head_left   = r_c + np.matmul(np.diag([-1, 1]), l * util.SC_vect(θ_c))
        head_center = r_c + R_robot*util.CS_vect(θ_c)

        head       = [[head_left, head_center],
                      [head_right, head_center],
                      [head_left, head_right]]
        head_color = np.array([(0.0, 0.0, 0.0, 0.8), (0.0, 0.1, 0.0, 0.8), (0.6, 0.1, 0.2, 0.8)])
        head_art = pltC.LineCollection(head, colors=(0.0, 0.1, 0.0, 0.8), linewidths=1.0)

        wheels = [[wheel_leftBack, wheel_leftFront],
                  [wheel_rightBack, wheel_rightFront]]
        wheel_color = np.array([(0, 0, 0, 0.85), (0, 0, 0, 0.85)])
        wheel_art = pltC.LineCollection(wheels, colors=wheel_color, linewidths=4)

        base1 = patches.Circle((r_c[0], r_c[1]),
                               R_robot,
                               color=(1.0, 0.0, 0.0, 0.7))
        base2 = patches.Circle((r_c[0], r_c[1]),
                               0.70 * R_robot,
                               color=(0.3, 0.7, 1.0, 1.0))

        wheel_art.set_zorder(0)
        base1.set_zorder(5)
        base2.set_zorder(10)
        head_art.set_zorder(20)

        art_1 = self._axes.add_collection(wheel_art)
        art_2 = self._axes.add_artist(base1)
        art_3 = self._axes.add_artist(base2)
        art_4 = self._axes.add_collection(head_art)
        art_list = [art_1, art_2, art_3, art_4]

        self._camera.snap(art_list)

    def refresh_figure(self):
        self._figure.show()

    def set_random_time_control(self, distribution='U'):
        (t_1, t_2) = self._robot.get_time_duration()
        dura = t_2 - t_1
        dt_min = (self._dt_head_min_pph / 100) * dura
        dt_max = (self._dt_head_max_pph / 100) * dura

        while True:
            if distribution == 'U':
                t_head_min = np.random.uniform(t_1, t_2)
                t_head_max = np.random.uniform(t_1, t_2)
            elif distribution == 'N':
                μ_tHeadControl_pph = self._μ_tHeadControl_pph
                Σ_tHeadControl_pph = self._Σ_tHeadControl_pph

                (μ_tMin, Σ_tMin)  = (np.array(μ_tHeadControl_pph)*dura,
                                     np.diag(Σ_tHeadControl_pph)*dura)

                t_head = np.random.multivariate_normal(μ_tMin, Σ_tMin, 1)
                t_head_min = t_head[0][0]
                t_head_max = t_head[0][1]
            else:
                print('\nERROR: no such distribution.\n')

            dt = t_head_max - t_head_min
            if (dt > dt_min) & (dt < dt_max) & (t_head_max >= 0) & (t_head_min >= 0):
                break

        self._robot.set_t_head_max(t_head_max)
        self._robot.set_t_head_min(t_head_min)
        return t_head_min, t_head_max

    def collision_trajectory(self, plot=False):
        r_cTilda   = self._xTilda[:, 0:2]
        (x_c, y_c) = util.polar2xy_large(r_cTilda)

        Δ_trajectory = self._Δ_trajectory

        x_c_approx = []
        y_c_approx = []
        for i in range(0, len(x_c), Δ_trajectory):
            x_c_approx.append(x_c[i])
            y_c_approx.append(y_c[i])

        for i in range(0, len(x_c_approx)):
            r_cTilda_approx_i = np.array([x_c_approx[i],
                                          y_c_approx[i]])
            collision_i = self.collision(r_cTilda_approx_i, plot)
            if collision_i:
                return True

        return False

    def sample_angle(self, n, distribution='U'):
        if distribution == 'U':
            θ_rand = np.random.uniform(-np.pi, np.pi, n)
        elif distribution == 'N':
            μ, σ = self._robot.get_x_0()[2], self._headSD_Guass  # mean and standard deviation
            θ_rand = np.random.normal(μ, σ, n)
        else:
            print('\nERROR: no such distribution.\n')

        return θ_rand

    def random_config(self, n, distribution='U'):
        (x_rand, y_rand) = self.sample(n, distribution)
        θ_rand           = self.sample_angle(n, distribution)

        if n == 1:
            (ρ_rand, φ_rand) = util.xy2polar(x_rand[0], y_rand[0])
            q_rand = np.array([ρ_rand, φ_rand, θ_rand[0]])
        else:
            (ρ_rand, φ_rand) = util.xy2polar_large(x_rand, y_rand)
            q_rand = np.stack([ρ_rand, φ_rand])
            q_rand = np.transpose(q_rand)

        return q_rand                               # O/P: q_random = [ρ φ θ] in radians

    def new_state(self, q_rand, x_near):
        r_ref_polar = q_rand[0:2]
        q_ref       = np.concatenate((r_ref_polar, [util.heading_direction(x_near[0:2], r_ref_polar)]), axis=0)
        self._robot.set_x_0(x_near)
        self._robot.set_q_ref(q_ref)

        msg     = self.draw_robot_trajectory(plot=False, draw_successful_trajectory=False)
        success = (msg == 'Integration successful.')
        if success:
            return not self.collision_trajectory(plot=False)

        return success

    def check_goal(self, q):
        ε = self._ε_goal

        r_goal = np.array(self._goal)
        (x, y) = util.polar2xy(q)
        r_new  = np.array([x, y])
        Δr     = r_goal - r_new

        distance = np.linalg.norm(Δr)
        goal = (distance < ε)
        return goal

    def get_goal_node_indices(self, v_goal):
        indices = []

        id_goal = v_goal.id()
        indices.append(id_goal)

        while id_goal != 0:
            v_goal = v_goal.get_parent()
            id_goal = v_goal.id()
            indices.append(id_goal)

        self._goal_indices = indices

    def get_goal_trajectory(self):
        if len(self._goal_indices) == 0:
            print('\nERROR: no goal yet.\n')

        self._xTilda = np.array([0, 0, 0, 0, 0])

        self._goal_indices.reverse()
        for i in range(0, len(self._goal_indices)-1):
            v_i          = self._RRTtree.get_vertex(self._goal_indices[i])
            v_iPlus1     = self._RRTtree.get_vertex(self._goal_indices[i+1])
            e_i_2_iPlus1 = self._RRTtree.get_edge(v_i, v_iPlus1)

            x_i                      = v_i.element()
            q_iPlus                  = v_iPlus1.get_reference_config()
            (t_head_min, t_head_max) = e_i_2_iPlus1.element()

            self._robot.set_x_0(x_i)
            self._robot.set_q_ref(q_iPlus)
            self._robot.set_t_head_min(t_head_min)
            self._robot.set_t_head_max(t_head_max)

            (xTilda, info) = self._robot.get_trajectory(degrees=False, plot=False)
            r_cTilda             = xTilda[:, 0:2]
            (x_c, y_c)           = util.polar2xy_large(r_cTilda)
            self._axes.plot(x_c, y_c, color=(0.0, 0, 1.0, 0.25), linestyle='-', linewidth=4.0)

            self._xTilda = np.vstack((self._xTilda, xTilda))

        self._xTilda[0, :] = self._xTilda[1, :]

    # _______________________________________________RRT___________________________________________________________
    def build_RRT(self, K):
        results = []
        for k in range(0, K):
            q_rand           = self.random_config(1, distribution='N')
            r_ref_polar      = q_rand[0:2]
            (x_rand, y_rand) = util.polar2xy(r_ref_polar)
            if self.collision([x_rand, y_rand], plot=True):
                results.append('bad sample')
                continue

            extended = self.extend_tree(q_rand)
            results.append(extended)

            if len(self._goal_indices) > 0:
                break
        return results

    def extend_tree(self, q_rand):              # only ρ and φ are random
        v_near = self.nearest_neighbor(q_rand)
        x_near = v_near.element()

        good_state = self.new_state(q_rand, x_near)
        if good_state:
            end   = len(self._xTilda[:, 1]) - 1
            x_new = self._xTilda[end, :]
            q_new = x_new[0:3]

            t_head_control = (self._robot.get_t_head_min(), self._robot.get_t_head_max())

            v_new = self._RRTtree.insert_vertex(x=x_new, padre=v_near)

            r_ref_polar = q_rand[0:2]
            q_ref = np.concatenate((r_ref_polar, [util.heading_direction(x_near[0:2], r_ref_polar)]), axis=0)
            v_new.set_reference_config(q_ref=q_ref)

            e_new = self._RRTtree.get_edge(v_near, v_new)
            e_new.set_element(t_head_control)

            metric = util.metric(q_new, q_rand, self._metric_weight)
            ε = self._ε
            if metric < ε:
                if self.check_goal(q_new):
                    print('\nGoal found!!!!!!! \n')
                    self.get_goal_node_indices(v_new)
                return 'reached'
            else:
                if self.check_goal(q_new):
                    print('\nGoal found!!!!!!! \n')
                    self.get_goal_node_indices(v_new)
                return 'advanced'
        return 'trapped'
    # _______________________________________________RRT___________________________________________________________

    def nearest_neighbor(self, q_rand, exact=False):
        distances = []
        for v in self._RRTtree.vertices():
            x = v.element()
            q = x[0:3]

            metric_i = util.metric(q, q_rand, self._metric_weight)
            distances.append(metric_i)
        min_index = np.argmin(distances)
        v_nearest = self._RRTtree.get_vertex(min_index)
        return v_nearest




