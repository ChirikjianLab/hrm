# HighwayRoadMap
A paradigm for robot motion planning based on parameterizations of the free space

## Description
We develop a motion planning paradigm based on the closed-form Minkowski sum and difference between ellipsoid and general obstacle (bounded as a convex differentiable surface, i.e. superquadrics). This repository is the C++ implementation of algorithms and benchmarks. The algorithms includes: Highway RoadMap (HRM) for both SE(2) and SE(3) rigid body planning problems, Hybrid Probabilistic Highway RoadMap (HP-HRM) for articulated body planning problems and Closed-Form Collision-Free ConFiguration (CF3) sampler for sampling-based planners. The algorithms have been compared with sampled-based planners from OMPL. The benchmark results show that our proposed methods outperform the sample-based planners (i.e. PRM, RRT, RRT-Connect, etc) especially in the narrow-passage problems.

## Dependencies
- [OMPL](https://ompl.kavrakilab.org/installation.html) (version >= 1.5.0): Open Motion Planning Library for sample-based planners
- [FCL](https://github.com/flexible-collision-library/fcl) (version = 0.6.0): Flexible Collision Library for collision detection
- [CGAL](https://www.cgal.org/) (version >= 5.2.1): Mesh generation as a pre-process
- [KDL](https://orocos.org/wiki/orocos/kdl-wiki.html): Kinematics and Dynamics Library for operations on kinematic chains
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) (version >= 2.91.0)
- [Boost](https://www.boost.org/) (version >= 1.55.0)
- [google-test](https://github.com/google/googletest) (version >= 1.10.x)
- (Optional) [KDL-parser](http://wiki.ros.org/kdl_parser): Parser from URDF to KDL
- [cmake-format] (version >= 0.4.5)
    ```
    sudo pip3 install cmake-format==0.4.5
    ```
- [clang-format]
    ```
    sudo apt install clang
    ```
- [Cppcheck]
    ```
    sudo apt install cppcheck
    ```

## Compiling Instructions
This is a standard CMake project, so compiling using the standard way of building is fine, i.e. from the root folder do the following commands:
```
mkdir build
cd build
cmake ../
make
```
The compilations are tested on Ubuntu 16.04 and 18.04 systems using CMake (>= 3.10). All the binaries are then located in "/bin" folder.

**Note**: 
- If you have installed OMPL from ROS, please make sure that its version is higher than 1.5.0, otherwise some features used in benchmark files might not be available. To link correct OMPL, you might need to add prefix when compiling, i.e.
```
cmake ../ -DCMAKE_PREFIX_PATH=/your/ompl/path -DOMPL_PREFIX=/your/ompl/include/dir
```
- We provide an installation script for dependencies: "${ROOT_DIR}/script/install-dependencies.sh". Exectuting it will automatically install all the required dependencies.

## Testing Instructions
### Generate configuration files
Configuration files for environment and robot when testing the C++ scripts are all stored in "/config" folder, including (3D cases as an example) "arena_config_3D.csv", "obs_config_3D.csv". "robot_config_3D.csv" and "end_points_3D.csv". In the repository, there are pre-defined configurations in these files. To customize different robot or environment configurations, simply run the Matlab scripts "/matlab/tests/planning_config_3D.m", and change different parameters that defines the geometric shapes of the obstacles and robots.

### Running tests
Testing files are located in "/test" folder:
- 2D case (single body, Kinematics of Containment): "TestHRM2D.cpp"
- 2D case (single body, Tighly-fitted Ellipsoid): "TestHRM2DTightFittedEllipse.cpp"
- 3D case (sinlge body): "TestHRM3D.cpp"
- 3D case (multi body): "TestHRM3DMultiBody.cpp"
- GTest for geometric subroutines: "/gtest/GTestGeometry.cpp"

### Benchmarks
Benchmark files are stored in "/test/benchmark" folder:
- 3D OMPL planners: "BenchOMPL3D.cpp" (for general superquadric shapes), "BenchOMPL3DEllipsoid.cpp" (for ellipsoidal shapes)
- 3D Highway multi body: "BenchHRM3DMultiBody.cpp"
- 3D Highway multi body with adaptive C-layers updates: "BenchHRM3DAdaptive.cpp"

### Visualizations
After running test or benchmark files, parameters for visualization will be generated in the "/bin" folder. Visualization scripts are all in Matlab:
- Plot 2D results: "/matlab/tests/plot_results_2D.m"
- Plot 2D Highway RoadMap graph: "/matlab/tests/plot_results_graph_2D.m"
- Plot 3D results: "/matlab/tests/plot_results_highway_3D.m"
- Plot 3D OMPL planners results: "/matlab/tests/plot_results_ompl_3D.m"

## Status
### Highway RoadMap planner
1. SE(2) single rigid body cases: 
- Fixed number of layers and sweep lines.
- Layer connections using KC ("src/planners/HighwayRoadMap.cpp") and TFE ("src/planners/HighwayRoadMap2d.cpp").

2. SE(3) single/multi rigid body cases:
- Single body ("src/planners/HighwayRoadMap3d.cpp"): fixed number of layers and sweep lines.
- Multi body: fixed ("src/planners/Hrm3DMultiBody.cpp") / adaptive ("src/planners/Hrm3DMultiBodyAdaptive.cpp") number of layers, fixed sweep lines.
- Layer connections using TFE.

### OMPL sampled-based planners
1. SE(3) single/multi rigid body:
- Benchmark: "/tests/benchmark/BenchOMPL3D.cpp"

### Benchmark in different scenerios
- Sparse map
- Cluttered map
- Maze map
- Home environment map

### Demo
1. Verified the algorithm in real robot, i.e. project the NAO humanoid robot to the plane, and plan a trajectory.

## Related Papers
"Ruan, S., Ma, Q., Poblete, K.L., Yan, Y. and Chirikjian G.S., 2018, December. Path Planning for Ellipsoidal Robots and General Obstacles via Closed-Form Characterization of Minkowski Operations. WAFR 2018."

"Yan, Y., Ma, Q. and Chirikjian, G.S., 2016, October. Path Planning Based on Closed-Form Characterization of Collision-Free Configuration-Spaces for Ellipsoidal Bodies, Obstacles, and Environments. In Proc. Int. Workshop Robot Learn. Plan. (pp. 13-19)."

"Yan, Y. and Chirikjian, G.S., 2015. Closed-form characterization of the Minkowski sum and difference of two ellipsoids. Geometriae Dedicata, 177(1), pp.103-128."

"Ruan, S., Ding, J., Ma, Q. and Chirikjian, G.S., 2019. The Kinematics of Containment for N-Dimensional Ellipsoids. Journal of Mechanisms and Robotics, 11(4), p.041005."

"Ruan, S., Ding, J. and Chirikjian, G.S., 2018. Lower Bounds of the Allowable Motions of One N-Dimensional Ellipsoid Contained in Another. ASME IDETC 2018."

"Ma, Q. and Chirikjian, G.S., 2015, August. A Closed-Form Lower Bound on the Allowable Motion for an Ellipsoidal Body and Environment. ASME IDETC 2015."

"Chirikjian, G.S. and Yan, Y., 2014. The Kinematics of Containment. In Advances in Robot Kinematics (pp. 355-364). Springer International Publishing."

