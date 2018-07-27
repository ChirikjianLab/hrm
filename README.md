# HighwayRoadMap
A paradigm for robot motion planning based on parameterizations of the free space

## Description
We develop a motion planning algorithm based on the closed-form Minkowski sum and difference between ellipsoids. Full algorithm is developed in Matlab and benchmark code is written in C++. The algorithm in SE(2) rigid body planning case has been compared with sampled-based planners from OMPL. The benchmark results show that our proposed method outperforms the sample-based planners (i.e. PRM, RRT, RRT-Connect, etc) especially in the narrow-passage problem.

## Plans
### Extend to 2D multi-body planning case
1. Update the algorithm to deal with multi-body problems, such as a "rabbit-shaped" or "snake-like" robot in 2D;
2. Verify the algorithm in real robot, i.e. project the NAO humanoid robot to the plane, and plan a trajectory.

### Extend to SE(3) rigid-body planning case
1. Hybrid with sample-based algorithms for the rotational component, and construct C-space for translational motions.

## Related Papers
"Yan, Y. and Chirikjian, G.S., 2015. Closed-form characterization of the Minkowski sum and difference of two ellipsoids. Geometriae Dedicata, 177(1), pp.103-128."

"Yan, Y., Ma, Q. and Chirikjian, G.S., 2016, October. Path Planning Based on Closed-Form Characterization of Collision-Free Configuration-Spaces for Ellipsoidal Bodies, Obstacles, and Environments. In Proc. Int. Workshop Robot Learn. Plan. (pp. 13-19)."

"Chirikjian, G.S. and Yan, Y., 2014. The Kinematics of Containment. In Advances in Robot Kinematics (pp. 355-364). Springer International Publishing."

"Ma, Q. and Chirikjian, G.S., 2015, August. A Closed-Form Lower Bound on the Allowable Motion for an Ellipsoidal Body and Environment. ASME IDETC 2015."

"Ruan, S., Ding, J. and Chirikjian, G.S., 2018. Lower Bounds of the Allowable Motions of One N-Dimensional Ellipsoid Contained in Another. ASME IDETC 2018."

