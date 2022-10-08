#pragma once

#include "geometry/MeshGenerator.h"
#include "geometry/SuperEllipse.h"

namespace hrm {

bool isCollision(const SuperQuadrics& object1,
                 fcl::CollisionObject<double> collisionObject1,
                 const SuperQuadrics& object2,
                 fcl::CollisionObject<double> collisionObject2);

bool isCollision(const SuperEllipse& object1,
                 fcl::CollisionObject<double> collisionObject1,
                 const SuperEllipse& object2,
                 fcl::CollisionObject<double> collisionObject2);

fcl::CollisionObject<double> setCollisionObjectFromSQ(
    const SuperQuadrics& object);

fcl::CollisionObject<double> setCollisionObjectFromSQ(
    const SuperEllipse& object);

}  // namespace hrm
