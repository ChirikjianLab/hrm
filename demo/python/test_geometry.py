"""
@author: Sipu Ruan
"""

import matplotlib.pyplot as plt

from MultiBodyTree import *


def test_sq():
    sq = SuperQuadrics([5.0, 3.0, 2.0], [0.3, 1.4], [-0.34, 1.34, 4.20], [-0.24, 2.04, 1.02, -3.21], 20)

    plt.figure()
    sq.plot()


def test_multi_body_tree():
    # Initialize MultiBodyTree object
    base = SuperQuadrics([5.0, 3.0, 2.0], [0.3, 1.4], [-0.34, 1.34, 4.20], [-0.24, 2.04, 1.02, -3.21], 20)
    robot = MultiBodyTree(base, 2)

    link = []
    link.append(SuperQuadrics([2.0, 1.5, 1.0], [0.78, 0.24], [0.32, 0.34, -4.20], [-1.24, 0.04, -1.2, 1.21], 20))
    link.append(SuperQuadrics([1.2, 1.0, 0.75], [0.1, 1.3], [-0.32, 0.24, 1.20], [-0.34, -0.04, 0.12, -0.1], 20))

    for i in range(robot.num_link):
        robot.add_body(link[i], i)

    plt.figure()
    ax = plt.axes(projection='3d')
    robot.plot(ax)

    # Transform robot
    g_base = SE3(10.0, -15.0, 20.0) * SE3.RPY([0.21, -0.3, 0.2])
    robot.transform(g_base)

    robot.plot(ax)


if __name__ == "__main__":
    test_sq()
    test_multi_body_tree()
    plt.show()
