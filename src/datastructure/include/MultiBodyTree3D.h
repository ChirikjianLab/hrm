#pragma once

#include "geometry/include/SuperQuadrics.h"
#include "util/include/ParseURDF.h"

#include <eigen3/Eigen/Geometry>

class MultiBodyTree3D {
  public:
    MultiBodyTree3D(SuperQuadrics base);

    /** \brief Getter functions */
    SuperQuadrics getBase() const { return base_; }
    std::vector<SuperQuadrics> getLinks() const { return link_; }
    std::vector<SuperQuadrics> getBodyShapes();

    double getNumLinks() const { return numLinks_; }
    std::vector<Eigen::Matrix4d> getTF() const { return tf_; }

    /** \brief Add a new body to the tree */
    void addBody(SuperQuadrics link);

    /** \brief Tranform robot */
    void robotTF(Eigen::Matrix4d tf);
    void robotTF(const std::string urdfFile, const Eigen::Matrix4d* gBase,
                 const Eigen::VectorXd* jointConfig);
    void robotTF(ParseURDF kdl, const Eigen::Matrix4d* gBase,
                 const Eigen::VectorXd* jointConfig);

    /**
     * \brief Closed-form Minkowski sums operation
     * \param s1 Object to be summed
     * \param k +1/-1 indicating sum/difference
     * \return A union of sampled points on the Minkowski sums boundary
     */
    std::vector<Eigen::MatrixXd> minkSum(const SuperQuadrics* s1, const int k);

  private:
    SuperQuadrics base_;
    double numLinks_ = 0;
    std::vector<SuperQuadrics> link_;
    std::vector<Eigen::Matrix4d> tf_;
};
