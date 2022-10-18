# hrm
![example workflow](https://github.com/ruansp/HighwayRoadMap/actions/workflows/github-actions-CI-basic.yml/badge.svg)
![example workflow](https://github.com/ruansp/HighwayRoadMap/actions/workflows/github-actions-CI-clang-tidy.yml/badge.svg)

The __Highway RoadMap (HRM)__ paradigm for robot motion planning based on parameterizations of the free space. This repository contains C++ implementation of algorithms and benchmarks for our paper in __IEEE Transactions on Robotics (T-RO)__.

### Authors
[Sipu Ruan](https://ruansp.github.io/), Karen L. Poblete, Hongtao Wu, Qianli Ma and [Gregory S. Chirikjian](https://cde.nus.edu.sg/me/staff/chirikjian-gregory-s/)

- Repository maintainers: Sipu Ruan, Qianli Ma

### Useful links
- Paper: [Link to T-RO](https://ieeexplore.ieee.org/document/9841604)
- Project page: [https://chirikjianlab.github.io/hrm-planning-page/](https://chirikjianlab.github.io/hrm-planning-page/)
- API documentation:

### Associate paper
- Ruan, S., Poblete, K.L., Wu, H., Ma, Q. and Chirikjian, G.S., 2022. Efficient Path Planning in Narrow Passages for Robots With Ellipsoidal Components. IEEE Transactions on Robotics. doi: 10.1109/TRO.2022.3187818
- BibTex
```
@article{ruan2022efficient,
  title={Efficient Path Planning in Narrow Passages for Robots With Ellipsoidal Components},
  author={Ruan, Sipu and Poblete, Karen L and Wu, Hongtao and Ma, Qianli and Chirikjian, Gregory S},
  journal={IEEE Transactions on Robotics},
  year={2022},
  publisher={IEEE}
}
```

## Description
We develop a motion planning paradigm based on the closed-form Minkowski sum and difference between ellipsoid and general obstacle (bounded as a convex differentiable surface, i.e. superquadrics). The algorithms includes: Highway RoadMap (HRM) for both SE(2) and SE(3) rigid body planning problems and a hybrid Probabilistic Highway RoadMap (Prob-HRM) for articulated body planning problems. The algorithms have been compared with sampled-based planners from OMPL. The benchmark results show that our proposed methods outperform the sample-based planners (i.e. PRM, RRT, RRT-Connect, etc) especially in the narrow-passage problems.

## Dependencies
We provide an installation script for dependencies: [`install-dependencies-on-localhost.sh`](/script/install-dependencies-on-localhost.sh). Exectuting it will automatically install all the following required dependencies:
- [OMPL](https://ompl.kavrakilab.org/installation.html) (version >= 1.5.0): Open Motion Planning Library for sampling-based planners
- [FCL](https://github.com/flexible-collision-library/fcl) (version >= 0.6.0): Flexible Collision Library for collision detection
- [CGAL](https://www.cgal.org/) (version >= 5.2.1): Mesh generation as a pre-process
- [KDL](https://orocos.org/wiki/orocos/kdl-wiki.html): Kinematics and Dynamics Library for operations on kinematic chains
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) (version >= 3.3.0)
- [Boost](https://www.boost.org/) (version >= 1.71.0)
- [google-test](https://github.com/google/googletest) (version >= 1.10)
- (Optional) [KDL-parser](http://wiki.ros.org/kdl_parser): Parser from URDF to KDL

## Compiling Instructions
### On Local Host
This is a standard CMake project, so compiling using the standard way of building is fine, i.e. from the root folder do the following commands:
```sh
mkdir build
cd build
cmake ../
make
```
**Note**:
- The compilation is tested on Ubuntu 20.04/22.04 systems using CMake (>= 3.10)
- If you have installed OMPL from ROS, please make sure that its version is higher than 1.5.0, otherwise some features used in benchmark files might not be available. To link correct OMPL, you might need to add prefix when compiling, i.e. 
```sh
cmake ../ -DOMPL_INCLUDE_DIRS=/your/ompl/include/dir
```

### On Docker (Recommended)
```sh
# Pull directory onto your localhost
cd $HOME && mkdir HighwayRoadMap_WS
cd HighwayRoadMap_WS && git clone git@github.com:ruansp/HighwayRoadMap.git

# Fetch docker image from remote and launch it locally (No need to build the image locally)
./HighwayRoadMap/script/start-docker.sh

# Build the project inside the docker container
cd HighwayRoadMap_WS
mkdir build
cd build
cmake -G Ninja ../HighwayRoadMap
ninja -j 9
```

## Installation and linking the library in another project
### Installation
After building:
```sh
[sudo] make install
```
By default, the library will be install in `/usr/local/`

To uninstall:
```sh
[sudo] make uninstall
```

### Linking in another package
In your `CMakeLists.txt`, add a line:
```
find_package(hrm REQUIRED)
```

To link the library to an executable (e.g., a demonstration for HRM planner in the 3D case):
```
add_executable(demo_hrm_3d demo_hrm_3d.cpp)
target_link_libraries(demo_hrm_3d
                      hrm::HRM3D
                      hrm::TestUtil)
```
Other linking library choices: `hrm::Geometry`, `hrm::DataStructure`, `hrm::HighwayRoadMap`, `hrm::HRM2D`, `hrm::OMPLPlanner`, `hrm::Util`

## Testing Instructions
### Generate configuration files
Configuration files for environment and robot when testing the C++ scripts are all stored in `/config` folder, including (3D cases as an example) `arena_config_3D.csv`, `obs_config_3D.csv`. `robot_config_3D.csv` and `end_points_3D.csv`.

In the repository, there are pre-defined configurations in these files, stored in [`/resources/`](/resources) folder. There are 2 ways to customize different robot or environment configurations:
- Run MATLAB scripts [`planning_config_3D.m`](/demo/matlab/planning_config_3D.m), and change different parameters that defines the geometric shapes of the obstacles and robots.
- Use C++ planning scene parser function defined [here](/include/hrm/test/util/ParsePlanningSettings.h#L197), i.e.,
```
hrm::parsePlanningConfig("superquadrics", "cluttered", "rabbit", "3D");
```

### Demonstration scripts
Demonstrations scripts are located in [`/demo`](/demo) folder:
- [`DemoHRMRigidBodyPlanning3D.cpp`](/demo/DemoHRMRigidBodyPlanning3D.cpp)
- [`DemoProbHRMArticulatedBodyPlanning3D.cpp`](/demo/DemoProbHRMArticulatedBodyPlanning3D.cpp)
- [`DemoOMPLRigidBodyPlanning3D.cpp`](/demo/DemoOMPLRigidBodyPlanning3D.cpp)
- [`DemoOMPLArticulatedBodyPlanning3D.cpp`](/demo/DemoOMPLArticulatedBodyPlanning3D.cpp)

Detailed results will be stored in `/result/details` folder and can be visualized using either MATLAB or Python scripts (See [here](README.md#visualizations) for running instructions).

### Running unit tests
```sh
cd build/
ctest
```

Testing files are located in [`/test`](/test) folder:
- Geometric subroutines: [`TestGeometry.cpp`](/test/TestGeometry.cpp)
- 2D HRM planner for a single elliptical body: [`TestHRM2D.cpp`](/test/TestHRM2D.cpp)
- 3D HRM planner for rigid body: [`TestHRM3D.cpp`](/test/TestHRM3D.cpp)
- 3D HRM planner for rigid body with ablated C-slice connections: [`TestHRM3DAblation.cpp`](/test/TestHRM3DAblation.cpp)
- 3D Prob-HRM planner for articulated body: [`TestProbHRM3D.cpp`](/test/TestProbHRM3D.cpp)
- 3D OMPL planners for rigid body: [`TestOMPL3D.cpp`](/test/TestOMPL3D.cpp)
- 3D OMPL planners for articulated body: [`TestOMPL3DArticulated.cpp`](/test/TestOMPL3DArticulated.cpp)

### Benchmarks
Benchmark files are stored in [`/test/benchmark/`](/test/benchmark) folder:
- 2D HRM planner for a single body: [`BenchHRM2D.cpp`](/test/benchmark/BenchHRM2D.cpp)
```sh
# Parameters: --Map type -- Robot type --Num of trials --Num of slices --[optional] Num of sweep lines
./BenchHRM2D sparse rabbit 50 20 30
```

- 3D HRM planner for rigid body: [`BenchHRM3D.cpp`](/test/benchmark/BenchHRM3D.cpp)
```sh
# Parameters: --Map type --Robot type --Num of trials --Max planning time --Num of slices --Method for pre-defined SO(3) samples --[optional] Num of sweep lines (x-direction) --[optional] Num of sweep lines (y-direction)
./BenchHRM3D sparse rabbit 50 60.0 60 icosahedron 6 3
```

- 3D HRM planner for rigid body (ablated version, without "bridge C-slice" process): [`BenchHRM3DAblation.cpp`](/test/benchmark/BenchHRM3DAblation.cpp)
```sh
# Parameters: --Map type --Robot type --Num of trials --Max planning time --Num of slices --Method for pre-defined SO(3) samples --[optional] Num of sweep lines (x-direction) --[optional] Num of sweep lines (y-direction)
./BenchHRM3DAblation sparse rabbit 50 60.0 60 icosahedron 6 3
```

- 3D Prob-HRM planner for articulated body: [`BenchProbHRM3D.cpp`](/test/benchmark/BenchProbHRM3D.cpp)
```sh
# Parameters: --Map type --Robot type --Num of trials --Max planning time (in seconds) --[optional] Num of sweep lines (x-direction) --[optional] Num of weep lines (y-direction)"
./BenchProbHRM3D sparse snake 50 60.0 6 3
```

- 3D Prob-HRM planner for articulated body (ablated version, without "bridge C-slice" process): [`BenchProbHRM3DAblation.cpp`](/test/benchmark/BenchProbHRM3DAblation.cpp)
```sh
# Parameters: --Map type --Robot type --Num of trials --Max planning time (in seconds) --[optional] Num of sweep lines (x-direction) --[optional] Num of weep lines (y-direction)"
./BenchProbHRM3DAblation sparse snake 50 60.0 6 3
```

- 3D OMPL planner for rigid body: [`BenchOMPL3D.cpp`](/test/benchmark/BenchOMPL3D.cpp)
```sh
# Parameters: --Map type --Robot type --Num of trials --Planner start ID --Planner end ID --Sampler start ID --Sampler end ID --Max planning time (in seconds)
./BenchOMPL3D sparse rabbit 50 0 5 0 4 60.0
```

- 3D OMPL planner for articulated body: [`BenchOMPL3DArticulated.cpp`](/test/benchmark/BenchOMPL3DArticulated.cpp)
```sh
# Parameters: --Map type --Robot type --Num of trials --Planner start ID --Planner end ID --Sampler start ID --Sampler end ID --Max planning time (in seconds)
./BenchOMPL3DArticulated sparse snake 50 0 5 0 4 60.0
```

**Note**:
- The SO(3) samples can be specified based on the generating methods. In the resources folder, we provide two methods: (1) "icosahedron": samples from icosahedral symmetry group (as described in paper); (2) "hopf": samples using Hopf fibration.
- The URDF file is only available for articulated-body planning problems, with the robot type being "snake" or "tree".

### Visualizations
After running demo or benchmark scripts, results for visualization will be generated in the `/result` folder. Visualization scripts are in both MATLAB and Python:

For MATLAB:
- Plot results from HRM planner (2D case): [`plot_results_hrm_2D.m`](/demo/matlab/plot_results_hrm_2D.m)
- Plot graph from HRM (2D case): [`plot_results_graph_2D.m`](/demo/matlab/plot_results_graph_2D.m)
- Plot results from HRM/Prob-HRM planners (3D case): [`plot_results_hrm_3D.m`](/demo/matlab/plot_results_hrm_3D.m)
- Plot graph from HRM (3D case): [`plot_results_graph_3D.m`](/demo/matlab/plot_results_graph_3D.m)
- Plot results from OMPL planners (3D case): [`plot_results_ompl_3D.m`](/demo/matlab/plot_results_ompl_3D.m)
- Generate movie for results from HRM planner (3D case): [`movie_results_hrm_3D.m`](/demo/matlab/movie_results_hrm_3D.m)

For Python (Please follow [`/demo/python/requirements.txt`](/demo/python/requirements.txt) for dependencies installation):
- Plot 3D HRM/Prob-HRM results and graph: [`plot_results_hrm_3D.py`](/demo/python/plot_results_hrm_3D.py)
- Plot 3D OMPL planners results and data structure: [`plot_results_ompl_3D.py`](/demo/python/plot_results_ompl_3D.py)

## Features
### Highway RoadMap planner
- SE(2) single rigid body:
  - C-slice connections using Tightly-Fitted Ellipsoid (TFE) [`HRM2D`](/include/hrm/planners/HRM2D.h) and Kinematics of Containment (KC) [`HRM2DKC`](/include/hrm/planners/HRM2DKC.h).

- SE(3) rigid body:
  - C-slice connections using TFE [`HRM3D`](/include/hrm/planners/HRM3D.h)
  - Ablated version for C-slice connections [`HRM3DAblication`](/include/hrm/planners/HRM3DAblation.h)

- 3D articulated body:
  - Probabilistic HighwayRoadMap [`ProbHRM3D`](/include/hrm/planners/ProbHRM3D.h)

### OMPL sampled-based planners
- SE(2) rigid body: [`OMPL2D`](/include/hrm/planners/ompl_interface/OMPL2D.h)
- SE(3) rigid body: [`OMPL3D`](/include/hrm/planners/ompl_interface/OMPL3D.h)
- 3D articulated body: [`OMPL3DArticulated`](/include/hrm/planners/ompl_interface/OMPL3DArticulated.h)

### Available robot types
- Rigid body: Rabbit, Chair
- Articulated body: Snake, Tree

### Available planning scene environments
Sparse, Cluttered, Maze, Home, Narrow

### Demonstration using real robot in a lab environment
- Verified the algorithm in real robot, i.e. project the NAO humanoid robot to the plane, and plan a trajectory.

## Other Related Papers
- Conference version:
  - "Ruan, S., Ma, Q., Poblete, K.L., Yan, Y. and Chirikjian G.S., 2018, December. Path Planning for Ellipsoidal Robots and General Obstacles via Closed-Form Characterization of Minkowski Operations. WAFR 2018."

- Initial algorithmic framework for ellipsoidal robot planning in scenes with ellipsoidal obstacles:
  - "Yan, Y., Ma, Q. and Chirikjian, G.S., 2016, October. Path Planning Based on Closed-Form Characterization of Collision-Free Configuration-Spaces for Ellipsoidal Bodies, Obstacles, and Environments. In Proc. Int. Workshop Robot Learn. Plan. (pp. 13-19)."

- Initial work on closed-form Minkowski sums and difference between two ellipsoids:
  - "Yan, Y. and Chirikjian, G.S., 2015. Closed-form characterization of the Minkowski sum and difference of two ellipsoids. Geometriae Dedicata, 177(1), pp.103-128."

- Works on Kinematics of Containment (KC):
  - "Ruan, S., Ding, J., Ma, Q. and Chirikjian, G.S., 2019. The Kinematics of Containment for N-Dimensional Ellipsoids. Journal of Mechanisms and Robotics, 11(4), p.041005."
  - "Ruan, S., Ding, J. and Chirikjian, G.S., 2018. Lower Bounds of the Allowable Motions of One N-Dimensional Ellipsoid Contained in Another. ASME IDETC 2018."
  - "Ma, Q. and Chirikjian, G.S., 2015, August. A Closed-Form Lower Bound on the Allowable Motion for an Ellipsoidal Body and Environment. ASME IDETC 2015."
  - "Chirikjian, G.S. and Yan, Y., 2014. The Kinematics of Containment. In Advances in Robot Kinematics (pp. 355-364). Springer International Publishing."

