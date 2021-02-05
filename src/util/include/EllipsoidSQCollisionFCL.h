#ifndef ELLIPSOIDSQCOLLISIONFCL_H
#define ELLIPSOIDSQCOLLISIONFCL_H

#include "MeshGenerator.h"
#include "geometry/include/SuperEllipse.h"

bool isCollision(const SuperQuadrics& obj1,
                 fcl::CollisionObject<double> colObj1,
                 const SuperQuadrics& obj2,
                 fcl::CollisionObject<double> colObj2);

bool isCollision(const SuperEllipse& obj1,
                 fcl::CollisionObject<double>* colObj1,
                 const SuperEllipse& obj2,
                 fcl::CollisionObject<double>* colObj2);

fcl::CollisionObject<double> setCollisionObjectFromSQ(
    const SuperQuadrics& object);

fcl::CollisionObject<double> setCollisionObjectFromSQ(
    const SuperEllipse& object);

#endif  // ELLIPSOIDSQCOLLISIONFCL_H
