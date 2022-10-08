#include "datastructure/MultiBodyTree2D.h"
#include "datastructure/MultiBodyTree3D.h"
#include "planners/HighwayRoadMap-inl.h"

template class hrm::planners::HighwayRoadMap<hrm::MultiBodyTree2D,
                                             hrm::SuperEllipse>;
template class hrm::planners::HighwayRoadMap<hrm::MultiBodyTree3D,
                                             hrm::SuperQuadrics>;
