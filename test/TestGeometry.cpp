#include "geometry/include/SuperEllipse.h"
#include "geometry/include/SuperQuadrics.h"
#include "geometry/include/TightFitEllipsoid.h"
#include "util/include/ExponentialFunction.h"

#include "gtest/gtest.h"

static const double epsilon = 1e-5;

// Tests for SuperEllipse
TEST(TestSuperEllipse, boundarySampling) {
    SuperEllipse S({5.0, 3.0}, 1.25, {-2.6, 3.2}, 0.0, 50);
    BoundaryPoints boundary = S.getOriginShape();

    // Point on major semi axis
    ASSERT_DOUBLE_EQ(boundary(0, 0), 2.4);
    ASSERT_DOUBLE_EQ(boundary(1, 0), 3.2);
}

TEST(TestSuperEllipse, minkowskiBoundarySampling) {
    SuperEllipse S({5.0, 3.0}, 1.25, {3.2, -2.5}, 0.0, 50);
    SuperEllipse E({2.5, 1.5}, 1.0, {0.0, 0.0}, 0.0, 50);
    BoundaryPoints minkBound = S.getMinkSum2D(E, +1);

    // Point on major semi axis
    ASSERT_DOUBLE_EQ(minkBound(0, 0), 10.7);
    ASSERT_DOUBLE_EQ(minkBound(1, 0), -2.5);
}

// Tests for SuperQuadrics
TEST(TestSuperQuadrics, boundarySampling) {
    SuperQuadrics S({5.0, 3.0, 2.0}, {1.25, 0.3}, {2.32, -1.5, 4.0},
                    Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0), 20);
    BoundaryPoints boundary = S.getOriginShape();

    // Point on semi axis
    for (auto i = 0; i < boundary.cols(); ++i) {
        if (std::fabs(boundary(0, i) - 2.32) < epsilon &&
            std::fabs(boundary(1, i) - (-1.5)) < epsilon) {
            EXPECT_TRUE(std::fabs(boundary(2, i) - 6.0) < epsilon ||
                        std::fabs(boundary(2, i) - 2.0) < epsilon);
        }
    }
}

TEST(TestSuperQuadrics, minkowskiBoundarySampling) {
    SuperQuadrics S({5.0, 3.0, 2.0}, {1.25, 0.3}, {-3.2, 3.24, -2.13},
                    Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0), 20);
    SuperQuadrics E({2.5, 1.5, 1.0}, {1.0, 1.0}, {0.0, 0.0, 0.0},
                    Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0), 20);
    BoundaryPoints minkBound = S.getMinkSum3D(E, +1);

    // Point on semi axis
    for (auto i = 0; i < minkBound.cols(); ++i) {
        if (std::fabs(minkBound(0, i) - (-3.2)) < epsilon &&
            std::fabs(minkBound(1, i) - 3.24) < epsilon) {
            EXPECT_TRUE(std::fabs(minkBound(2, i) - (3.0 - 2.13)) < epsilon ||
                        std::fabs(minkBound(2, i) - (-3.0 - 2.13)) < epsilon);
        }
    }
}

// Test for TightFittedEllipsoid
TEST(TestTightFittedEllipsoid, mvce2D) {
    SuperEllipse mvce = getMVCE2D({5.0, 3.0}, {5.0, 3.0}, 0.0, pi / 2, 50);

    EXPECT_TRUE(std::fabs(mvce.getSemiAxis().at(0) - 5.0) < epsilon);
    EXPECT_TRUE(std::fabs(mvce.getSemiAxis().at(1) - 5.0) < epsilon);
}

TEST(TestTightFittedEllipsoid, mvce3D) {
    SuperQuadrics mvce = getMVCE3D({5.0, 3.0, 2.0}, {2.0, 3.0, 5.0},
                                   Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0),
                                   Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0), 20);

    EXPECT_TRUE(std::fabs(mvce.getSemiAxis().at(0) - 3.0) < epsilon);
    EXPECT_TRUE(std::fabs(mvce.getSemiAxis().at(1) - 5.0) < epsilon);
    EXPECT_TRUE(std::fabs(mvce.getSemiAxis().at(2) - 5.0) < epsilon);
}

int main(int ac, char* av[]) {
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
