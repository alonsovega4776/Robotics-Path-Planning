import Obstacle
import Environment
import Robot
import Tree
import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate

O_0 = Obstacle.Obstacle([(0, 0),
                         (0.5, 0.0),
                         (0.5, 1.0),
                         (0.3, 0.3)],
                        convex=True)

O_1 = Obstacle.Obstacle([(5.0, 6.0),
                         (0.6, 8.2),
                         (1.7, 5.5)],
                        convex=True)

O_2 = Obstacle.Obstacle([(2.0, 2.0),
                         (3.0, 0.5),
                         (4.0, 2.0),
                         (3.0, 4.0)],
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

env = Environment.Environment([0, 10], [0, 10], [O_0, O_1, O_2, O_3, O_4], (2, 1), (8, 8))

#env.print_environment()
#env.sample(100, 'Ν')
#env.refresh_figure()

x_0   = np.array([0.0,    np.radians(40.5),    np.radians(0),      0, 0])
q_ref = np.array([4.0,    np.radians(0),        np.radians(0)])
t_1 = 0
t_2 = 5.0
N = 2000
robo = Robot.Robot(x_0, q_ref, t_1, t_2, N)

#sol = robo.get_trajectory()

T = Tree.Tree([0, 0])









