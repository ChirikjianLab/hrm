from result_loader import load_results, load_planning_scene
from robot_generator import generate_robot, plot_robot_pose
from SuperQuadrics import SuperQuadrics
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


def main():
    result_path = "../../result/details/"
    config_path = "../../config/"
    resource_path = "../../resources/3D/"
    dim = "3D"

    # Load planning scene and results
    print("Loading planning scene and results...")
    robot_config, arena_config, obstacle_config, end_pts = load_planning_scene(dim, config_path)
    x_origin, x_mink, cf_seg, vtx, edge, path = load_results(dim, result_path)

    # Generate robot object
    print("Loading robot...")
    size_config = end_pts.shape
    urdf_file = None
    if size_config[1] == 10:
        urdf_file = resource_path + "urdf/snake.urdf"
    elif size_config[1] == 16:
        urdf_file = resource_path + "urdf/tri-snake.urdf"

    robot, robot_urdf = generate_robot(robot_config, urdf_file)

    # Plot
    print("Display the results...")
    fig = plt.figure()
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
        for i in range(0, len(path), int(len(path)/5)):
            plot_robot_pose(robot, path[i, 0:7], robot_urdf, ax)

    plt.show()


if __name__ == "__main__":
    main()
    