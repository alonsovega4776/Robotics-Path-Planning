import Obstacle
import Environment
import Robot
import Tree
import BinaryTree
import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate
import quaternion
import Utility as util


O_0 = Obstacle.Obstacle([(0.0, 1.0),
                         (1.5, 2.0),
                         (0.5, 2.0),
                         (0.3, 1.3)],
                        convex=True)

O_1 = Obstacle.Obstacle([(5.0, 6.0),
                         (0.6, 8.2),
                         (1.7, 5.5)],
                        convex=True)

O_2 = Obstacle.Obstacle([(2.0, 3.0),
                         (3.0, 3.5),
                         (4.0, 3.0),
                         (3.0, 5.5)],
                        convex=True)

O_3 = Obstacle.Obstacle([(6.0, 2.0),
                         (9.0, 4.5),
                         (8.0, 0.0),
                         (7.0, 0.2)],
                        convex=True)

O_4 = Obstacle.Obstacle([(6.0, 5.0),
                         (7.0, 6.5),
                         (4.0, 9.0),
                         (5.5, 10.0)],
                        convex=True)
#"""    Testing Environment _____________________________________________
env = Environment.Environment([0, 10], [0, 10], [O_0, O_1, O_2, O_3, O_4], (2, 1), (8, 8))
env.print_environment()

x_0   = np.array([1.0,    np.radians(40.0),    np.radians(80.0),      0, 0])
r_ref_polar = [9.0,    np.radians(90.0)]
q_ref = np.concatenate((r_ref_polar, [util.heading_direction(x_0[0:2], r_ref_polar)]), axis=0)
t_1   = 0.0
t_2   = 2.0
N     = 100

env.get_robot().set_x_0(x_0)
env.get_robot().set_q_ref(q_ref)
env.get_robot().set_time_duration(t_1, t_2)
env.get_robot().set_number_time_steps(N)
(t1_control, t2_control) = env.set_random_time_control('N')

env.draw_robot_trajectory(plot=False)
traj_coll = env.collision_trajectory(plot=True)

ani = env.play_robot_trajectory()

#"""  # Testing Environment _____________________________________________







