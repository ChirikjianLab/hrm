"""
@author: Sipu Ruan
"""

from result_loader import load_results, load_planning_scene
from robot_generator import generate_robot, plot_robot_pose
from SuperQuadrics import SuperQuadrics
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d
import os


def plot_results_hrm_3d(is_plot_graph=False):
    curr_dir = os.getcwd()
    result_path = curr_dir + "/../../result/details/"
    config_path = curr_dir + "/../../config/"
    resource_path = curr_dir + "/../../resources/3D/"
    method = "hrm"
    dim = "3D"

    # Load planning scene
    print("Loading planning scene and results...")
    robot_config, arena_config, obstacle_config, end_pts = load_planning_scene(dim, config_path)

    # Generate robot object
    print("Loading robot...")
    size_config = end_pts.shape
    urdf_file = None
    if size_config[1] == 10:
        method = "prob_hrm"
        urdf_file = resource_path + "urdf/snake.urdf"
    elif size_config[1] == 16:
        method = "prob_hrm"
        urdf_file = resource_path + "urdf/tree.urdf"

    robot, robot_urdf = generate_robot(robot_config, urdf_file)

    # Load planning results
    x_origin, x_mink, cf_seg, vtx, edge, path = load_results(method + "_" + dim, result_path)



    # Plot
    print("Figure: Display the results...")
    plt.figure()
    ax = plt.axes(projection="3d")

    # Planning environment
    obstacle = [SuperQuadrics()] * len(obstacle_config)
    for i in range(len(obstacle_config)):
        obstacle[i] = SuperQuadrics(obstacle_config[i, 0:3], obstacle_config[i, 3:5],
                                    obstacle_config[i, 5:8], obstacle_config[i, 8:12], 20)
        obstacle[i].plot(ax, 'y')

    # Robot start/goal configurations
    plot_robot_pose(robot, end_pts[0], robot_urdf, ax)
    plot_robot_pose(robot, end_pts[1], robot_urdf, ax)

    ax.plot3D(end_pts[0, 0], end_pts[0, 1], end_pts[0, 2], 'ro', linewidth=2)
    ax.plot3D(end_pts[1, 0], end_pts[1, 1], end_pts[1, 2], 'go', linewidth=2)

    # Planned path
    if len(path) > 0:
        ax.plot3D([end_pts[0, 0], path[0, 0]], [end_pts[0, 1], path[0, 1]], [end_pts[0, 2], path[0, 2]],
                  'gray', linewidth=2)
        ax.plot3D(path[:, 0], path[:, 1], path[:, 2], 'gray', linewidth=2)
        ax.plot3D([end_pts[1, 0], path[-1, 0]], [end_pts[1, 1], path[-1, 1]], [end_pts[1, 2], path[-1, 2]],
                  'gray', linewidth=2)

        # Robot following the path
        for i in range(0, len(path)):
            plot_robot_pose(robot, path[i, :], robot_urdf, ax)

    # Plot graph and sweep line in one layer
    if is_plot_graph:
        print("Figure: Show the graph...")
        plt.figure()
        ax = plt.axes(projection="3d")

        for i in range(len(obstacle_config)):
            obstacle[i] = SuperQuadrics(obstacle_config[i, 0:3], obstacle_config[i, 3:5],
                                        obstacle_config[i, 5:8], obstacle_config[i, 8:12], 20)
            obstacle[i].plot(ax, 'y')

        # Graph vertex
        ax.plot3D(vtx[:, 0], vtx[:, 1], vtx[:, 2], '.k')

        # Graph edge
        for i in range(edge.shape[0]):
            idx_start = int(edge[i, 0])
            idx_end = int(edge[i, 1])

            ax.plot3D([vtx[idx_start, 0], vtx[idx_end, 0]], [vtx[idx_start, 1], vtx[idx_end, 1]],
                      [vtx[idx_start, 2], vtx[idx_end, 2]], '-k')

        # Plot sweep line at one layer
        print("Figure: Show sweep lines at one layer...")
        plt.figure()
        ax = plt.axes(projection="3d")

        # C-obstacle boundary
        for i in range(0, x_mink.shape[0]-3*(robot.num_link+1), 3):
            num = int(np.sqrt(x_mink.shape[1]))
            xx = np.reshape(x_mink[i], (num, num))
            yy = np.reshape(x_mink[i+1], (num, num))
            zz = np.reshape(x_mink[i+2], (num, num))

            ax.plot_surface(xx, yy, zz, color="b", alpha=0.3)

        # Sweep lines
        for i in range(cf_seg.shape[0]):
            ax.plot3D([cf_seg[i, 0], cf_seg[i, 0]], [cf_seg[i, 1], cf_seg[i, 1]], [cf_seg[i, 2], cf_seg[i, 4]], '-m')
            ax.plot3D(cf_seg[i, 0], cf_seg[i, 1], cf_seg[i, 2], '.m')
            ax.plot3D(cf_seg[i, 0], cf_seg[i, 1], cf_seg[i, 4], '.m')

            ax.plot3D(cf_seg[i, 0], cf_seg[i, 1], cf_seg[i, 3], '.k')


if __name__ == "__main__":
    plot_results_hrm_3d(True)
    plt.show()
