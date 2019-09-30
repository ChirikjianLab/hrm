#ifndef HRM3DMULTIADAPTIVE_H
#define HRM3DMULTIADAPTIVE_H

#include "Hrm3dMultiBody.h"

#define pi 3.1415926

class Hrm3DMultiBodyAdaptive : public Hrm3DMultiBody {
public:
  Hrm3DMultiBodyAdaptive(MultiBodyTree3D, std::vector<std::vector<double>>,
                         std::vector<SuperQuadrics>, std::vector<SuperQuadrics>,
                         option3D);

  void planPath(double);
  void connectMultiLayer();
  virtual ~Hrm3DMultiBodyAdaptive();
};

#endif // HRM3DMULTIADAPTIVE_H
