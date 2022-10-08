import matplotlib.pyplot as plt

from MultiBodyTree import MultiBodyTree
from SuperQuadrics import SuperQuadrics
import roboticstoolbox as rtb
from spatialmath import SE3, UnitQuaternion


def generate_robot(robot_config, urdf_file=None):
    num = 20
    robot_urdf = None

    # The base body
    size_robot_config = robot_config.shape
    num_link = size_robot_config[0] - 1

    base = SuperQuadrics(robot_config[0, 0:3], robot_config[0, 3:5], robot_config[0, 5:8], robot_config[0, 8:12], num)
    robot = MultiBodyTree(base, num_link, urdf_file)

    # Robot links
    for i in range(num_link):
        robot.add_body(SuperQuadrics(robot_config[i+1, 0:3], robot_config[i+1, 3:5],
                                     robot_config[i+1, 5:8], robot_config[i+1, 8:12], num), i)

    # For articulated bodies
    if urdf_file is not None:
        robot_urdf = rtb.ERobot.URDF(urdf_file)
        if num_link != robot_urdf.n:
            print("Number of bodies in URDF and shape files not match...")

    return robot, robot_urdf


def plot_robot_pose(robot, pose_config, robot_urdf=None, ax=None):
    rot = UnitQuaternion(pose_config[3:7])
    pose = SE3(pose_config[0:3]) * rot.SE3()

    # Transform the robot
    if robot_urdf is None:
        robot.transform(pose)
    else:
        robot.transform(pose, pose_config[7:])

    # Display robot
    if ax is None:
        ax = plt.axes(projection="3d")
    robot.plot(ax)
