
import SpatialGraph

from scipy.spatial import ConvexHull, convex_hull_plot_2d
import numpy as np
import matplotlib.pyplot as plt

G = SpatialGraph.Graph(directed=False)

v_0 = G.insert_vertex(1.0, 1.5, None)
v_1 = G.insert_vertex(2.0, 0.5, None)
v_2 = G.insert_vertex(0.5, 12.5, None)
v_3 = G.insert_vertex(3.5, 12.5, None)

print(G.distance(v_2, v_2))


points = np.array([[0.2, 0.2], [0.2, 0.4], [0.5, 0.6], [0.25, 0.3]])
hull = ConvexHull(points, incremental=True)

plt.plot(points[:, 0], points[:, 1], 'o')

for simplex in hull.simplices:
    plt.plot(points[simplex, 0], points[simplex, 1], 'k-')
plt.plot(points[hull.vertices,0], points[hull.vertices,1], 'r--', lw=2)

plt.plot(points[hull.vertices[0],0], points[hull.vertices[0],1], 'ro')
plt.show()

hull.add_points([[0.1, 0.6]])
hull.close()



