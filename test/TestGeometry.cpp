#include "geometry/include/SuperEllipse.h"
#include "geometry/include/SuperQuadrics.h"
#include "geometry/include/TightFitEllipsoid.h"
#include "util/include/ExponentialFunction.h"

#include "gtest/gtest.h"

static const double pi = 3.1415926;
static const double epsilon = 1e-5;

// Tests for SuperEllipse
TEST(TestSuperEllipse, boundarySampling) {
    const SuperEllipse S({5.0, 3.0}, 1.25, {-2.6, 3.2}, 0.0, 50);
    const BoundaryPoints boundary = S.getOriginShape();

    // Point on major semi axis
    const double BOUNDARY_X = 2.4;
    const double BOUNDARY_Y = 3.2;
    ASSERT_DOUBLE_EQ(boundary(0, 0), BOUNDARY_X);
    ASSERT_DOUBLE_EQ(boundary(1, 0), BOUNDARY_Y);
}

TEST(TestSuperEllipse, minkowskiBoundarySampling) {
    const SuperEllipse S({5.0, 3.0}, 1.25, {3.2, -2.5}, 0.0, 50);
    const SuperEllipse E({2.5, 1.5}, 1.0, {0.0, 0.0}, 0.0, 50);
    const BoundaryPoints minkBound = S.getMinkSum2D(E, +1);

    // Point on major semi axis
    const double BOUNDARY_X = 10.7;
    const double BOUNDARY_Y = -2.5;
    ASSERT_DOUBLE_EQ(minkBound(0, 0), BOUNDARY_X);
    ASSERT_DOUBLE_EQ(minkBound(1, 0), BOUNDARY_Y);
}

// Tests for SuperQuadrics
TEST(TestSuperQuadrics, boundarySampling) {
    const SuperQuadrics S({5.0, 3.0, 2.0}, {1.25, 0.3}, {2.32, -1.5, 4.0},
                          Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0), 20);
    const BoundaryPoints boundary = S.getOriginShape();

    // Point on semi axis
    const double BOUNDARY_X = 2.32;
    const double BOUNDARY_Y = -1.5;
    for (auto i = 0; i < boundary.cols(); ++i) {
        if (std::fabs(boundary(0, i) - BOUNDARY_X) < epsilon &&
            std::fabs(boundary(1, i) - BOUNDARY_Y) < epsilon) {
            EXPECT_TRUE(std::fabs(boundary(2, i) - 6.0) < epsilon ||
                        std::fabs(boundary(2, i) - 2.0) < epsilon);
        }
    }
}

TEST(TestSuperQuadrics, minkowskiBoundarySampling) {
    const SuperQuadrics S({5.0, 3.0, 2.0}, {1.25, 0.3}, {-3.2, 3.24, -2.13},
                          Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0), 20);
    const SuperQuadrics E({2.5, 1.5, 1.0}, {1.0, 1.0}, {0.0, 0.0, 0.0},
                          Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0), 20);
    const BoundaryPoints minkBound = S.getMinkSum3D(E, +1);

    // Point on semi axis
    const double BOUNDARY_X = -3.2;
    const double BOUNDARY_Y = 3.24;
    for (auto i = 0; i < minkBound.cols(); ++i) {
        if (std::fabs(minkBound(0, i) - BOUNDARY_X) < epsilon &&
            std::fabs(minkBound(1, i) - BOUNDARY_Y) < epsilon) {
            EXPECT_TRUE(std::fabs(minkBound(2, i) - (3.0 - 2.13)) < epsilon ||
                        std::fabs(minkBound(2, i) - (-3.0 - 2.13)) < epsilon);
        }
    }
}

// Test for TightFittedEllipsoid
TEST(TestTightFittedEllipsoid, mvce2D) {
    const SuperEllipse mvce =
        getMVCE2D({5.0, 3.0}, {5.0, 3.0}, 0.0, pi / 2, 50);

    const double SEMI_1 = 5.0;
    const double SEMI_2 = 5.0;
    EXPECT_TRUE(std::fabs(mvce.getSemiAxis().at(0) - SEMI_1) < epsilon);
    EXPECT_TRUE(std::fabs(mvce.getSemiAxis().at(1) - SEMI_2) < epsilon);
}

TEST(TestTightFittedEllipsoid, mvce3D) {
    const SuperQuadrics mvce =
        getMVCE3D({5.0, 3.0, 2.0}, {2.0, 3.0, 5.0},
                  Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0),
                  Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0), 20);

    const double SEMI_1 = 3.0;
    const double SEMI_2 = 5.0;
    const double SEMI_3 = 5.0;
    EXPECT_TRUE(std::fabs(mvce.getSemiAxis().at(0) - SEMI_1) < epsilon);
    EXPECT_TRUE(std::fabs(mvce.getSemiAxis().at(1) - SEMI_2) < epsilon);
    EXPECT_TRUE(std::fabs(mvce.getSemiAxis().at(2) - SEMI_3) < epsilon);
}

int main(int ac, char* av[]) {
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
