#ifndef PROBHRM3D_H
#define PROBHRM3D_H

#include "HighwayRoadMap3d.h"
#include "util/include/ParseURDF.h"

class ProbHRM3D : public HighwayRoadMap3D {
  public:
    ProbHRM3D(const MultiBodyTree3D& robot, const std::string urdfFile,
              const std::vector<SuperQuadrics>& arena,
              const std::vector<SuperQuadrics>& obs,
              const PlanningRequest& req);

    virtual ~ProbHRM3D() override;

  public:
    void plan(const double timeLim);

    void connectMultiLayer() override;

    void generateVertices(const double tx,
                          const FreeSegment2D* freeSeg) override;

  protected:
    void setTransform(const std::vector<double>& v) override;

    void computeTFE(const std::vector<double>& v1,
                    const std::vector<double>& v2,
                    std::vector<SuperQuadrics>* tfe);

  private:
    ParseURDF* kdl_;
    std::string urdfFile_;
    const double maxJointAngle_ = pi / 2;

    // store configuration for each robot shape (at each C-layer)
    std::vector<std::vector<double>> v_;
};

#endif  // PROBHRM3D_H
