#ifndef PROBHRM3D_H
#define PROBHRM3D_H

#include "HRM3DMultiBody.h"
#include "util/include/ParseURDF.h"

class ProbHRM3D : public HRM3DMultiBody {
  public:
    ProbHRM3D(const MultiBodyTree3D& robot, const std::string urdfFile,
              const std::vector<SuperQuadrics>& arena,
              const std::vector<SuperQuadrics>& obs,
              const PlanningRequest& req);

    virtual ~ProbHRM3D() override;

  public:
    void plan(double timeLim);

    virtual void connectMultiLayer() override;

    virtual void generateVertices(const double tx,
                                  const cf_cell2D* cellYZ) override;

  protected:
    virtual void setTransform(const std::vector<double>& V) override;

    std::vector<SuperQuadrics> tfeArticulated(const std::vector<double>& v1,
                                              const std::vector<double>& v2);

  private:
    ParseURDF* kdl_;
    std::string urdfFile_;
    const double maxJointAngle_ = pi / 2;

    // store configuration for each robot shape (at each C-layer)
    std::vector<std::vector<double>> v_;
};

#endif  // PROBHRM3D_H
