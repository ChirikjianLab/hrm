"""
@author: Sipu Ruan
"""

import pandas as pd
import numpy as np


def load_planning_scene(dim, config_path):
    # Planning environment
    arena_config = pd.read_csv(config_path + "arena_config_" + dim + ".csv", header=None, dtype=np.float64).values
    obstacle_config = pd.read_csv(config_path + "obstacle_config_" + dim + ".csv", header=None, dtype=np.float64).values

    # Robot configurations
    robot_config = pd.read_csv(config_path + "robot_config_" + dim + ".csv", header=None, dtype=np.float64).values
    end_pts = pd.read_csv(config_path + "end_points_" + dim + ".csv", header=None, dtype=np.float64).values

    return robot_config, arena_config, obstacle_config, end_pts


def load_results(suffix, result_path):
    try:
        x_origin = pd.read_csv(result_path + "origin_bound_" + suffix + ".csv",
                               delimiter='\s+', header=None, dtype=np.float64).values
        x_mink = pd.read_csv(result_path + "mink_bound_" + suffix + ".csv",
                             delimiter='\s+', header=None, dtype=np.float64).values
        cf_seg = pd.read_csv(result_path + "segment_" + suffix + ".csv", header=None, dtype=np.float64).values
    except:
        x_origin = None
        x_mink = None
        cf_seg = None

        print('No discrete points on obstacles will be shown.')

    vtx = pd.read_csv(result_path + "vertex_" + suffix + ".csv", header=None, dtype=np.float64).values
    edge = pd.read_csv(result_path + "edge_" + suffix + ".csv", header=None, dtype=np.float64).values

    try:
        path = pd.read_csv(result_path + "interpolated_path_" + suffix + ".csv", header=None, dtype=np.float64).values
    except:
        path = pd.read_csv(result_path + "solution_path_" + suffix + ".csv", header=None, dtype=np.float64).values

    return x_origin, x_mink, cf_seg, vtx, edge, path
