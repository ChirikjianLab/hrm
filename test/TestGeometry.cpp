#include "geometry/SuperEllipse.h"
#include "geometry/SuperQuadrics.h"
#include "geometry/TightFitEllipsoid.h"
#include "util/ExponentialFunction.h"

#include "gtest/gtest.h"

// Tests for SuperEllipse
TEST(TestSuperEllipse, BoundarySampling) {
    const hrm::SuperEllipse S({5.0, 3.0}, 1.25, {-2.6, 3.2}, 0.0, 50);
    const hrm::BoundaryPoints boundary = S.getOriginShape();

    // Point on major semi axis
    const double boundaryX = 2.4;
    const double boundaryY = 3.2;
    ASSERT_DOUBLE_EQ(boundary(0, 0), boundaryX);
    ASSERT_DOUBLE_EQ(boundary(1, 0), boundaryY);
}

TEST(TestSuperEllipse, MinkowskiBoundarySampling) {
    const hrm::SuperEllipse S({5.0, 3.0}, 1.25, {3.2, -2.5}, 0.0, 50);
    const hrm::SuperEllipse E({2.5, 1.5}, 1.0, {0.0, 0.0}, 0.0, 50);
    const hrm::BoundaryPoints minkBound = S.getMinkSum2D(E, +1);

    // Point on major semi axis
    const double boundaryX = 10.7;
    const double boundaryY = -2.5;
    ASSERT_DOUBLE_EQ(minkBound(0, 0), boundaryX);
    ASSERT_DOUBLE_EQ(minkBound(1, 0), boundaryY);
}

// Tests for SuperQuadrics
TEST(TestSuperQuadrics, BoundarySampling) {
    const hrm::SuperQuadrics S({5.0, 3.0, 2.0}, {1.25, 0.3}, {2.32, -1.5, 4.0},
                               Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0), 20);
    const hrm::BoundaryPoints boundary = S.getOriginShape();

    // Point on semi axis
    const double boundaryX = 2.32;
    const double boundaryY = -1.5;
    for (auto i = 0; i < boundary.cols(); ++i) {
        if (std::fabs(boundary(0, i) - boundaryX) < hrm::EPSILON &&
            std::fabs(boundary(1, i) - boundaryY) < hrm::EPSILON) {
            EXPECT_TRUE(std::fabs(boundary(2, i) - 6.0) < hrm::EPSILON ||
                        std::fabs(boundary(2, i) - 2.0) < hrm::EPSILON);
        }
    }
}

TEST(TestSuperQuadrics, MinkowskiBoundarySampling) {
    const hrm::SuperQuadrics S({5.0, 3.0, 2.0}, {1.25, 0.3},
                               {-3.2, 3.24, -2.13},
                               Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0), 20);
    const hrm::SuperQuadrics E({2.5, 1.5, 1.0}, {1.0, 1.0}, {0.0, 0.0, 0.0},
                               Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0), 20);
    const hrm::BoundaryPoints minkBound = S.getMinkSum3D(E, +1);

    // Point on semi axis
    const double boundaryX = -3.2;
    const double boundaryY = 3.24;
    for (auto i = 0; i < minkBound.cols(); ++i) {
        if (std::fabs(minkBound(0, i) - boundaryX) < hrm::EPSILON &&
            std::fabs(minkBound(1, i) - boundaryY) < hrm::EPSILON) {
            EXPECT_TRUE(
                std::fabs(minkBound(2, i) - (3.0 - 2.13)) < hrm::EPSILON ||
                std::fabs(minkBound(2, i) - (-3.0 - 2.13)) < hrm::EPSILON);
        }
    }
}

// Test for TightFittedEllipsoid
TEST(TestTightFittedEllipsoid, MVCE2D) {
    const hrm::SuperEllipse mvce =
        hrm::getMVCE2D({5.0, 3.0}, {5.0, 3.0}, 0.0, hrm::HALF_PI, 50);

    const double semiAxis1 = 5.0;
    const double semiAxis2 = 5.0;
    EXPECT_TRUE(std::fabs(mvce.getSemiAxis().at(0) - semiAxis1) < hrm::EPSILON);
    EXPECT_TRUE(std::fabs(mvce.getSemiAxis().at(1) - semiAxis2) < hrm::EPSILON);
}

TEST(TestTightFittedEllipsoid, MVCE3D) {
    const hrm::SuperQuadrics mvce =
        hrm::getMVCE3D({5.0, 3.0, 2.0}, {2.0, 3.0, 5.0},
                       Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0),
                       Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0), 20);

    const double semiAxis1 = 3.0;
    const double semiAxis2 = 5.0;
    const double semiAxis3 = 5.0;
    EXPECT_TRUE(std::fabs(mvce.getSemiAxis().at(0) - semiAxis1) < hrm::EPSILON);
    EXPECT_TRUE(std::fabs(mvce.getSemiAxis().at(1) - semiAxis2) < hrm::EPSILON);
    EXPECT_TRUE(std::fabs(mvce.getSemiAxis().at(2) - semiAxis3) < hrm::EPSILON);
}

int main(int ac, char* av[]) {
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
