# HighwayRoadMap.resources
Pre-defined robot configurations, environmental features, etc.

# Folder structure
All configurations are stored in .csv files. Subfolders contain:
- `2D/` and `3D/`: store robot shape, map information and planning settings.
- `SO3_sequence/`: store pre-defined SO(3) samples using different sampling algorithms.
- `mesh/`: mesh information of environment
- `urdf/`: URDF file for multi-body robot

```
2D/
-- robot_{type}_2D.csv
-- env_{shape}_{map_type}_2D_arena.csv
-- env_{shape}_{map_type}_2D_obstacle.csv
-- setting_{shape}_{map_type}_2D.csv
3D/
-- robot_{type}_3D.csv
-- env_{shape}_{map_type}_3D_arena.csv
-- env_{shape}_{map_type}_3D_obstacle.csv
-- setting_{shape}_{map_type}_3D.csv
SO3_sequence/
-- q_{sample_type}_{num_samples}.csv
mesh/
urdf/
```

# Name conventions
Important note: for 3D rotations in `2D/` and `3D/` folders, we use the __Axis-Angle__ parameterization to as the storage convention; while in `SO3_sequence/` folder, the storage convention is __Unit-Quaternion__.

## Robot shape
Each line includes all parameters to define an ellipsoid (ellipse in 2D): semi axis lengths, exponents (= 1), center in Cartesian coordinates, orientation. The first line defines the base links, and the rests define other links (positions and orientations are __all relative to the base link__).
- 2D: `2D/robot_{type}_2D.csv`, each line: semi_axis_a, semi_axis_b, epsilon = 1, center_x, center_y, rotation_angle
- 3D: `3D/robot_{type}_3D.csv`, each line: semi_axis_a, semi_axis_b, semi_axis_c, epsilon_1 = 1, epsilon_2 = 1, center_x, center_y, center_z, rotation_axis_x, rotation_axis_y, rotation_axis_z, rotation_angle

`{type}`: name of the robot (i.e. rabbit, S-shape, etc).

## Environmental features
Arena and obstacles are stored in different files. Each line includes all parameters to define a superquadric (superellipse in 2D): semi axis lengths, exponents, center in Cartesian coordinates, orientation.
- 2D: `2D/env_{ellipse/superellipse}_{map_type}_2D_{arena/obstacle}.csv`, each line: semi_axis_a, semi_axis_b, epsilon, center_x, center_y, rotation_angle
- 3D: `3D/env_{ellipsoid/superquadrics}_{map_type}_3D_{arena/obstacle}.csv`, each line: semi_axis_a, semi_axis_b, semi_axis_c, epsilon_1, epsilon_2, center_x, center_y, center_z, rotation_axis_x, rotation_axis_y, rotation_axis_z, rotation_angle

## Planning settings
Definition of start and end poses in the planning problem.
- 2D: `2D/setting_{ellipse/superellipse}_{map_type}_2D.csv`, each line: center_x, center_y, rotation_angle
- 3D: `3D/setting_{ellipsoid/superquadrics}_{map_type}_3D.csv`, each line: center_x, center_y, center_z, rotation_axis_x, rotation_axis_y, rotation_axis_z, rotation_angle

`{map_type}`: type of the map (i.e. sparse, cluttered, maze, etc).
