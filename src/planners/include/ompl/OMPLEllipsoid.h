#ifndef OMPLELLIPSOID_H
#define OMPLELLIPSOID_H

#include "OMPL3D.h"

class OMPLEllipsoid : public OMPL3D {
  public:
    OMPLEllipsoid(std::vector<double> lowBound, std::vector<double> highBound,
                  const std::vector<SuperQuadrics>& robot,
                  const std::vector<SuperQuadrics>& arena,
                  const std::vector<SuperQuadrics>& obs,
                  const std::vector<Mesh>& obsMesh, const int planner,
                  const int sampler);
    ~OMPLEllipsoid() override;

  public:
    bool isStateValid(const ob::State* state) const override;

  private:
    bool checkSeparationASC(const SuperQuadrics& robotOrigin,
                            const SuperQuadrics& robotAux,
                            const SuperQuadrics& obs) const;
};

#endif  // OMPLELLIPSOID_H
