import Obstacle
import Environment
import Robot
import Tree
import BinaryTree
import Utility as util

import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate
import quaternion



O_0 = Obstacle.Obstacle([(0.0, 1.0),
                         (1.5, 2.0),
                         (0.5, 2.0),
                         (0.3, 1.3)],
                        convex=True)

O_1 = Obstacle.Obstacle([(4.0, 6.0),
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
start = (4.5, 4.5)
start_polar = util.xy2polar(start[0], start[1])
x_0   = np.concatenate((start_polar, [np.radians(80.0), 0, 0]))

env = Environment.Environment([0, 10], [0, 10], [O_0, O_1, O_2, O_3, O_4], x_0, (8, 8))
env.print_environment()

r_ref_polar = [9.0,    np.radians(90.0)]
q_ref = np.concatenate((r_ref_polar, [util.heading_direction(x_0[0:2], r_ref_polar)]), axis=0)
t_1   = 0.0
t_2   = 2.0
N     = 100

env.get_robot().set_q_ref(q_ref)
env.get_robot().set_time_duration(t_1, t_2)
env.get_robot().set_number_time_steps(N)



results = env.build_RRT(1000)
env.get_goal_trajectory()
ani = env.play_robot_trajectory()


#"""  # Testing Environment _____________________________________________







