# HighwayRoadMap
A paradigm for robot motion planning based on parameterizations of the free space

## Description
We develop a motion planning algorithm based on the closed-form Minkowski sum and difference between ellipsoid and general obstacle (bounded as a convex differentiable surface, i.e. superquadrics). Full algorithm is developed in Matlab and benchmark code is written in C++. The algorithms for both SE(2) and SE(3) rigid body planning problems have been developed and compared with sampled-based planners from OMPL. The benchmark results show that our proposed method outperforms the sample-based planners (i.e. PRM, RRT, RRT-Connect, etc) especially in the narrow-passage problem.

## Dependencies
- OMPL: Open Motion Planning Library for sample-based planners
- FCL: Flexible Collision Library for collision detection
- CCD: Required dependency for FCL
- CGAL: Mesh generation as a pre-process
- GMP: Required dependency for CGAL
- Eigen
- Boost

## Status
### Highway RoadMap planner
1. SE(2) single rigid body finished: 
- Fixed number of layers and sweep lines.
- Layer connections using KC ("src/planners/HighwayRoadMap.cpp") and TFE ("src/planners/HighwayRoadMap2d.cpp").

2. SE(3) single/multi rigid body in progress:
- Single body ("src/planners/HighwayRoadMap3d.cpp"): fixed number of layers and sweep lines.
- Multi body: fixed ("src/planners/Hrm3DMultiBody.cpp") / adaptive ("src/planners/Hrm3DMultiBodyAdaptive.cpp") number of layers, fixed sweep lines.
- Layer connections using TFE.

### OMPL sampled-based planners
1. SE(2) single rigid body

2. SE(3) single/multi rigid body

## TODO
### Benchmark in different scenerios
- Sparse map
- Cluttered map
- Narrow passage with different window size

### Articulated body
1. Extend to multi-link articulated robot

### Demo
1. Verify the algorithm in real robot, i.e. project the NAO humanoid robot to the plane, and plan a trajectory.

## Related Papers
"Ruan, S., Ma, Q., Poblete, K.L., Yan, Y. and Chirikjian G.S., 2018, December. Path Planning for Ellipsoidal Robots and General Obstacles via Closed-Form Characterization of Minkowski Operations. WAFR 2018."

"Yan, Y., Ma, Q. and Chirikjian, G.S., 2016, October. Path Planning Based on Closed-Form Characterization of Collision-Free Configuration-Spaces for Ellipsoidal Bodies, Obstacles, and Environments. In Proc. Int. Workshop Robot Learn. Plan. (pp. 13-19)."

"Yan, Y. and Chirikjian, G.S., 2015. Closed-form characterization of the Minkowski sum and difference of two ellipsoids. Geometriae Dedicata, 177(1), pp.103-128."

"Ruan, S., Ding, J., Ma, Q. and Chirikjian, G.S., 2019. The Kinematics of Containment for N-Dimensional Ellipsoids. Journal of Mechanisms and Robotics, 11(4), p.041005."

"Ruan, S., Ding, J. and Chirikjian, G.S., 2018. Lower Bounds of the Allowable Motions of One N-Dimensional Ellipsoid Contained in Another. ASME IDETC 2018."

"Ma, Q. and Chirikjian, G.S., 2015, August. A Closed-Form Lower Bound on the Allowable Motion for an Ellipsoidal Body and Environment. ASME IDETC 2015."

"Chirikjian, G.S. and Yan, Y., 2014. The Kinematics of Containment. In Advances in Robot Kinematics (pp. 355-364). Springer International Publishing."

