from SuperQuadrics import *
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np


class MultiBodyTree:
    def __init__(self, base, num_link, urdf_file=None):
        self.base = base
        self.num_link = num_link
        self.link = [SuperQuadrics()] * self.num_link
        self.tf = [SE3()] * self.num_link

        self.urdf_file = urdf_file
        self.robot_urdf = None

        # For articulated robot
        if self.urdf_file is not None:
            self.robot_urdf = rtb.ERobot.URDF(urdf_file)

    def add_body(self, link, i):
        if i > self.num_link:
            print("Link exceeds defined number, try again.")
        else:
            self.link[i] = link
            self.tf[i] = self.link[i].pose

    def set_base_transform(self, g):
        self.base.pose = SE3(g)

    def set_link_transform(self, link_id, g):
        self.link[link_id].pose = SE3(g)

    def transform(self, g_base, joint_config=None):
        self.set_base_transform(g_base)

        # Obtain pose of each link
        if self.robot_urdf is None:
            for i in range(len(self.link)):
                g_link = np.matmul(g_base.A, self.tf[i].A)
                self.set_link_transform(i, g_link)
        else:
            g_links = self.robot_urdf.fkine_all(joint_config)

            for i in range(len(self.link)):
                # Offset from body frame to ellipsoid center
                g_link = g_base * g_links[i] * self.tf[i]
                self.set_link_transform(i, g_link)

    def plot(self, ax):
        self.base.plot(ax)
        for link in self.link:
            link.plot(ax)
