"""
@author: Sipu Ruan
"""

import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3, UnitQuaternion
import matplotlib.pyplot as plt


def exp_func(x, epsilon):
    return np.sign(x) * (np.abs(x) ** epsilon)


class SuperQuadrics:
    def __init__(self, a=None, eps=None, tc=None, q=None, num=20):
        # Shape
        if a is None:
            a = [1.0, 1.0, 1.0]
        if eps is None:
            eps = [1.0, 1.0]
        self.a = a
        self.eps = eps

        # Pose
        if q is None:
            q = [1.0, 0.0, 0.0, 0.0]
        if tc is None:
            tc = [0.0, 0.0, 0.0]
        rot = UnitQuaternion(q)
        self.pose = SE3(tc) * rot.SE3()

        # Parameters for surface point samples
        self.num = num
        self.omega, self.eta = np.meshgrid(np.linspace(-np.pi-1e-6, np.pi+1e-6, self.num),
                                           np.linspace(-np.pi/2.0 - 1e-6, np.pi/2.0 + 1e-6, self.num))

    def get_points(self):
        xx = self.a[0] * exp_func(np.cos(self.eta), self.eps[0]) * exp_func(np.cos(self.omega), self.eps[1])
        yy = self.a[1] * exp_func(np.cos(self.eta), self.eps[0]) * exp_func(np.sin(self.omega), self.eps[1])
        zz = self.a[2] * exp_func(np.sin(self.eta), self.eps[0]) * np.ones(self.omega.shape)

        num_pts = np.size(xx)
        pts = np.empty((3, num_pts))
        pts[0, :] = np.reshape(xx, (1, num_pts))
        pts[1, :] = np.reshape(yy, (1, num_pts))
        pts[2, :] = np.reshape(zz, (1, num_pts))

        pts_transformed = np.matmul(self.pose.R, pts) + np.reshape(self.pose.t, (3, 1))

        return pts_transformed

    def get_surf(self):
        surf_pts = self.get_points()

        xx = np.reshape(surf_pts[0, :], (self.num, self.num))
        yy = np.reshape(surf_pts[1, :], (self.num, self.num))
        zz = np.reshape(surf_pts[2, :], (self.num, self.num))

        return xx, yy, zz

    def plot(self, ax=None, color=None):
        xx, yy, zz = self.get_surf()

        if ax is None:
            ax = plt.axes(projection='3d')

        try:
            surf = ax.plot_surface(xx, yy, zz, color=color)
        except:
            surf = ax.plot_surface(xx, yy, zz)

        ax.set_aspect('equal')

        return surf
