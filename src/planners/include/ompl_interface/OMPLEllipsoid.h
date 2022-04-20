#pragma once

#include "OMPL3D.h"

class OMPLEllipsoid : public OMPL3D {
  public:
    OMPLEllipsoid(std::vector<Coordinate> lowBound,
                  std::vector<Coordinate> highBound,
                  const std::vector<SuperQuadrics>& robot,
                  const std::vector<SuperQuadrics>& arena,
                  const std::vector<SuperQuadrics>& obs,
                  const std::vector<Mesh>& obsMesh, const Index planner,
                  const Index sampler);
    ~OMPLEllipsoid() override;

    bool isStateValid(const ob::State* state) const override;

  private:
    bool checkSeparationASC(const SuperQuadrics& robotOrigin,
                            const SuperQuadrics& robotAux,
                            const SuperQuadrics& obs) const;
};
