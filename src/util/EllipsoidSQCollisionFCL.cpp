#include "include/EllipsoidSQCollisionFCL.h"

using GeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;

bool isCollision(const SuperQuadrics& obj1,
                 fcl::CollisionObject<double> colObj1,
                 const SuperQuadrics& obj2,
                 fcl::CollisionObject<double> colObj2) {
    colObj1.setTransform(
        obj1.getQuaternion().toRotationMatrix(),
        fcl::Vector3d(obj1.getPosition().at(0), obj1.getPosition().at(1),
                      obj1.getPosition().at(2)));
    colObj2.setTransform(
        obj2.getQuaternion().toRotationMatrix(),
        fcl::Vector3d(obj2.getPosition().at(0), obj2.getPosition().at(1),
                      obj2.getPosition().at(2)));

    fcl::CollisionRequest<double> request;
    fcl::CollisionResult<double> result;
    fcl::collide(&colObj1, &colObj2, request, result);

    return result.isCollision();
}

bool isCollision(const SuperEllipse& obj1,
                 fcl::CollisionObject<double>* colObj1,
                 const SuperEllipse& obj2,
                 fcl::CollisionObject<double>* colObj2) {
    colObj1->setTransform(
        Eigen::Quaterniond(Eigen::AngleAxis<double>(
                               obj1.getAngle(), Eigen::Vector3d(0.0, 0.0, 1.0)))
            .toRotationMatrix(),
        fcl::Vector3d(obj1.getPosition().at(0), obj1.getPosition().at(1), 0.1));

    colObj2->setTransform(
        Eigen::Quaterniond(Eigen::AngleAxis<double>(
                               obj2.getAngle(), Eigen::Vector3d(0.0, 0.0, 1.0)))
            .toRotationMatrix(),
        fcl::Vector3d(obj2.getPosition().at(0), obj2.getPosition().at(1), 0.1));

    fcl::CollisionRequest<double> request;
    fcl::CollisionResult<double> result;
    fcl::collide(colObj1, colObj2, request, result);

    return result.isCollision();
}

fcl::CollisionObject<double> setCollisionObjectFromSQ(
    const SuperQuadrics& object) {
    if (std::fabs(object.getEpsilon().at(0) - 1.0) < 1e-6 &&
        std::fabs(object.getEpsilon().at(1) - 1.0) < 1e-6) {
        // Ellipsoid model
        GeometryPtr_t ellipsoid(new fcl::Ellipsoid<double>(
            object.getSemiAxis().at(0), object.getSemiAxis().at(1),
            object.getSemiAxis().at(2)));

        return fcl::CollisionObject<double>(ellipsoid);
    }

    // Mesh model
    Mesh objMesh = getMeshFromSQ(object);
    auto* model = new fcl::BVHModel<fcl::OBBRSS<double>>();
    model->beginModel();
    model->addSubModel(objMesh.vertices, objMesh.triangles);
    model->endModel();

    return fcl::CollisionObject<double>(GeometryPtr_t(model));
}

fcl::CollisionObject<double> setCollisionObjectFromSQ(
    const SuperEllipse& object) {
    if (std::fabs(object.getEpsilon() - 1.0) < 1e-6) {
        // Ellipsoid model
        GeometryPtr_t ellipse(new fcl::Ellipsoid<double>(
            object.getSemiAxis().at(0), object.getSemiAxis().at(1), 0.1));

        return fcl::CollisionObject<double>(ellipse);
    }

    // Mesh model
    SuperQuadrics objAux(
        {object.getSemiAxis().at(0), object.getSemiAxis().at(1), 0.1},
        {object.getEpsilon(), 0.1}, {0.0, 0.0, 0.0},
        Eigen::Quaterniond::Identity(), 10);
    Mesh objMesh = getMeshFromSQ(objAux);
    auto* model = new fcl::BVHModel<fcl::OBBRSS<double>>();
    model->beginModel();
    model->addSubModel(objMesh.vertices, objMesh.triangles);
    model->endModel();

    return fcl::CollisionObject<double>(GeometryPtr_t(model));
}
