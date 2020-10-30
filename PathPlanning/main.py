import Obstacle
import Environment
import matplotlib.pyplot as plt

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

env = Environment.Environment([0,10], [0,10], [O_0, O_1, O_2],(0,1),(10,10))



env.print_environment()

print(env.collision([1, 2]))














