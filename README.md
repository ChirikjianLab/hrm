# HighwayRoadMap
A paradigm for robot motion planning based on parameterizations of the free space

## Description
We develop a motion planning algorithm based on the closed-form Minkowski sum and difference between ellipsoids. Full algorithm is developed in Matlab and benchmark code is written in C++. The algorithm in SE(2) rigid body planning case has been compared with sampled-based planners from OMPL. The benchmark results show that our proposed method outperforms the sample-based planners (i.e. PRM, RRT, RRT-Connect, etc) especially in the narrow-passage problem.

## Status
### Highway RoadMap planner
1. SE(2) single rigid body finished: 
(1) Fixed number of layers and sweep lines.
(2) Layer connections using KC ("src/planners/highwayroadmap.cpp") and TFE ("src/planners/highwayroadmap2d.cpp").

2. SE(3) single/multi rigid body in progress:
(1) Single body ("src/planners/highwayroadmap3d.cpp"): fixed number of layers and sweep lines.
(2) Multi body: fixed ("src/planners/hrm3d_multibody.cpp") / adaptive ("src/planners/hrm3d_multi_adaptive.cpp") number of layers, fixed sweep lines.
(3) Layer connections using TFE.

### OMPL sampled-based planners
1. SE(2) single rigid body
2. SE(3) single/multi rigid body

## Plans
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

