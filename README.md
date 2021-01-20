# HighwayRoadMap
A paradigm for robot motion planning based on parameterizations of the free space

## Description
We develop a motion planning paradigm based on the closed-form Minkowski sum and difference between ellipsoid and general obstacle (bounded as a convex differentiable surface, i.e. superquadrics). This repository is the C++ implementation of algorithms and benchmarks. The algorithms for both SE(2) and SE(3) rigid body planning problems have been developed and compared with sampled-based planners from OMPL. The benchmark results show that our proposed method outperforms the sample-based planners (i.e. PRM, RRT, RRT-Connect, etc) especially in the narrow-passage problem.

## Dependencies
- [OMPL](https://ompl.kavrakilab.org/installation.html) (version >= 1.4.0): Open Motion Planning Library for sample-based planners
- [FCL](https://github.com/flexible-collision-library/fcl) (version >= 0.6.0): Flexible Collision Library for collision detection
- [CCD](https://github.com/danfis/libccd) (version >= 2.0): Required dependency for FCL
- [CGAL](https://www.cgal.org/): Mesh generation as a pre-process
- GMP: Required dependency for CGAL
- [KDL](https://orocos.org/wiki/orocos/kdl-wiki.html): Kinematics and Dynamics Library for operations on kinematic chains
- [KDL-parser](http://wiki.ros.org/kdl_parser): Parser from URDF to KDL
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) (version >= 2.91.0)
- [Boost](https://www.boost.org/) (version >= 1.55.0)
- [cmake-format] 
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
The compilations are tested on Ubuntu 16.04 system using CMake (>= 3.10). All the binaries are then located in "/bin" folder.

## Testing Instructions
### Generate configuration files
Configuration files are all stored in "/config" folder, including (3D cases as an example) "arena_config_3d.csv", "obs_config_3d.csv". "robot_config_3d.csv" and "endPts_3d.csv". In the repository, there are pre-defined configurations in these files. To customize different robot or environment configurations, simply run the Matlab scripts "/matlab/test/robotConfig_3D.m", and change different parameters that defines the geometric shapes.

### Running tests
Testing files are located in "/test" folder:
- 2D case (single body, Kinematics of Containment): "/test/TestHRM2D.cpp"
- 2D case (single body, Tighly-fitted Ellipsoid): "/test/TestHRM2DTightFittedEllipse.cpp"
- 3D case (sinlge body): "/test/TestHRM3D.cpp"
- 3D case (multi body): "/test/TestHRM3DMultiBody.cpp"
- GTest for geometric subroutines: "/test/unit-test/GTestGeometry.cpp"

### Benchmarks
Benchmark files are stored in "/benchmark" folder:
- 3D OMPL planners: "/benchmark/BenchOMPL3D.cpp", 
- 3D Highway single body: "/benchmark/BenchHRM3D.cpp"
- 3D Highway multi body: "/benchmark/BenchHRM3DMultiBody.cpp"
- 3D Highway multi body with adaptive C-layers updates: "/benchmark/BenchHRM3DAdaptive.cpp"

### Visualizations
After running test or benchmark files, parameters for visualization will be generated in the "/bin" folder. Visualization scripts are all in Matlab:
- Plot 2D results: "/matlab/tests/plotShape.m"
- Plot 3D single body results: "/matlab/tests/plotShape_3D.m"
- Plot 3D multi body results: "/matlab/tests/plotShape_3D_MultiBody.m"
- Plot 3D OMPL planners results: "/matlab/tests/plotShape_ompl_3D.m"

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
1. SE(2) single rigid body

2. SE(3) single/multi rigid body:
- Benchmark: "/benchmark/BenchOMPL3D.cpp"

### Benchmark in different scenerios
- Sparse map
- Cluttered map
- Maze map

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

