#include "datastructure/include/MultiBodyTree2D.h"
#include "datastructure/include/MultiBodyTree3D.h"
#include "include/HighwayRoadMap-inl.h"

template class hrm::planners::HighwayRoadMap<hrm::MultiBodyTree2D,
                                             hrm::SuperEllipse>;
template class hrm::planners::HighwayRoadMap<hrm::MultiBodyTree3D,
                                             hrm::SuperQuadrics>;
