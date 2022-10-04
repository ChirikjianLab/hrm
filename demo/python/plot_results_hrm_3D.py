from result_loader import load_results
from robot_generator import generate_robot


def main():
    result_path = "../../result/details/"
    config_path = "../../config/"
    resource_path = "../../resources/3D/"

    # Load results
    x_origin, x_mink, cf_seg, vtx, edge, path, robot_config, end_pts = load_results("3D", result_path, config_path)

    # Robot
    size_config = end_pts.shape
    urdf_file = None
    if size_config[1] == 10:
        urdf_file = resource_path + "urdf/snake.urdf"
    elif size_config[1] == 16:
        urdf_file = resource_path + "urdf/tri-snake.urdf"

    robot, robot_urdf = generate_robot(robot_config, urdf_file)


if __name__ == "__main__":
    main()