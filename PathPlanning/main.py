import Obstacle
import Environment
import Robot
import Tree
import BinaryTree
import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate
import quaternion

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

x_0   = np.array([1.0,    np.radians(45.0),    np.radians(-90.0),      0, 0])
q_ref = np.array([4.0,    np.radians(90),      np.radians(100.0)])
t_1   = 0.0
t_2   = 2.0
N     = 100

env.get_robot().set_x_0(x_0)
env.get_robot().set_q_ref(q_ref)
env.get_robot().set_time_duration(t_1, t_2)
env.get_robot().set_number_time_steps(N)


env.play_robot_trajectory()
env.refresh_figure()


#"""  # Testing Environment _____________________________________________







