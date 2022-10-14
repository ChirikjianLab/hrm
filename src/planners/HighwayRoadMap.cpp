#include "hrm/datastructure/MultiBodyTree2D.h"
#include "hrm/datastructure/MultiBodyTree3D.h"
#include "hrm/planners/HighwayRoadMap-inl.h"

template class hrm::planners::HighwayRoadMap<hrm::MultiBodyTree2D,
                                             hrm::SuperEllipse>;
template class hrm::planners::HighwayRoadMap<hrm::MultiBodyTree3D,
                                             hrm::SuperQuadrics>;
