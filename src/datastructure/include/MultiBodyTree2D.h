#pragma once

#include "geometry/include/SuperEllipse.h"

#include "eigen3/Eigen/Geometry"

class MultiBodyTree2D {
  public:
    MultiBodyTree2D(SuperEllipse base);

    /** \brief Getter functions */
    SuperEllipse getBase() const { return base_; }
    std::vector<SuperEllipse> getLinks() const { return link_; }
    double getNumLinks() const { return numLinks_; }
    std::vector<Eigen::Matrix3d> getTF() const { return tf_; }

    /** \brief Add a new body to the tree */
    void addBody(SuperEllipse link);

    /** \brief Tranform robot */
    void robotTF(Eigen::Matrix3d tf);

    /**
     * \brief Closed-form Minkowski sums operation
     * \param s1 Object to be summed
     * \param k +1/-1 indicating sum/difference
     * \return A union of sampled points on the Minkowski sums boundary
     */
    std::vector<Eigen::MatrixXd> minkSum(const SuperEllipse* s1, const int k);

  private:
    SuperEllipse base_;
    double numLinks_ = 0;
    std::vector<SuperEllipse> link_;
    std::vector<Eigen::Matrix3d> tf_;
};
