#include "include/EllipsoidSQCollisionFCL.h"

using GeometryPtr = std::shared_ptr<fcl::CollisionGeometry<double>>;

bool hrm::isCollision(const SuperQuadrics& object1,
                      fcl::CollisionObject<double> collisionObject1,
                      const SuperQuadrics& object2,
                      fcl::CollisionObject<double> collisionObject2) {
    collisionObject1.setTransform(
        object1.getQuaternion().toRotationMatrix(),
        fcl::Vector3d(object1.getPosition().at(0), object1.getPosition().at(1),
                      object1.getPosition().at(2)));
    collisionObject2.setTransform(
        object2.getQuaternion().toRotationMatrix(),
        fcl::Vector3d(object2.getPosition().at(0), object2.getPosition().at(1),
                      object2.getPosition().at(2)));

    fcl::CollisionRequest<double> request;
    fcl::CollisionResult<double> result;
    fcl::collide(&collisionObject1, &collisionObject2, request, result);

    return result.isCollision();
}

bool hrm::isCollision(const SuperEllipse& object1,
                      fcl::CollisionObject<double> collisionObject1,
                      const SuperEllipse& object2,
                      fcl::CollisionObject<double> collisionObject2) {
    collisionObject1.setTransform(
        Eigen::Quaterniond(
            Eigen::AngleAxis<double>(object1.getAngle(),
                                     Eigen::Vector3d(0.0, 0.0, 1.0)))
            .toRotationMatrix(),
        fcl::Vector3d(object1.getPosition().at(0), object1.getPosition().at(1),
                      0.1));

    collisionObject2.setTransform(
        Eigen::Quaterniond(
            Eigen::AngleAxis<double>(object2.getAngle(),
                                     Eigen::Vector3d(0.0, 0.0, 1.0)))
            .toRotationMatrix(),
        fcl::Vector3d(object2.getPosition().at(0), object2.getPosition().at(1),
                      0.1));

    fcl::CollisionRequest<double> request;
    fcl::CollisionResult<double> result;
    fcl::collide(&collisionObject1, &collisionObject2, request, result);

    return result.isCollision();
}

fcl::CollisionObject<double> hrm::setCollisionObjectFromSQ(
    const SuperQuadrics& object) {
    if (std::fabs(object.getEpsilon().at(0) - 1.0) < 1e-6 &&
        std::fabs(object.getEpsilon().at(1) - 1.0) < 1e-6) {
        // Ellipsoid model
        const GeometryPtr ellipsoid(new fcl::Ellipsoid<double>(
            object.getSemiAxis().at(0), object.getSemiAxis().at(1),
            object.getSemiAxis().at(2)));

        return fcl::CollisionObject<double>(ellipsoid);
    }

    // Mesh model
    const Mesh objMesh = getMeshFromSQ(object);
    auto modelPtr = std::make_shared<fcl::BVHModel<fcl::OBBRSS<double>>>();
    modelPtr->beginModel();
    modelPtr->addSubModel(objMesh.vertices, objMesh.triangles);
    modelPtr->endModel();

    return fcl::CollisionObject<double>(GeometryPtr(modelPtr));
}

fcl::CollisionObject<double> hrm::setCollisionObjectFromSQ(
    const SuperEllipse& object) {
    if (std::fabs(object.getEpsilon() - 1.0) < 1e-6) {
        // Ellipsoid model
        const GeometryPtr ellipse(new fcl::Ellipsoid<double>(
            object.getSemiAxis().at(0), object.getSemiAxis().at(1), 0.1));

        return fcl::CollisionObject<double>(ellipse);
    }

    // Mesh model
    const SuperQuadrics objAux(
        {object.getSemiAxis().at(0), object.getSemiAxis().at(1), 0.1},
        {object.getEpsilon(), 0.1}, {0.0, 0.0, 0.0},
        Eigen::Quaterniond::Identity(), 10);
    const Mesh objMesh = getMeshFromSQ(objAux);
    auto modelPtr = std::make_shared<fcl::BVHModel<fcl::OBBRSS<double>>>();
    modelPtr->beginModel();
    modelPtr->addSubModel(objMesh.vertices, objMesh.triangles);
    modelPtr->endModel();

    return fcl::CollisionObject<double>(GeometryPtr(modelPtr));
}
