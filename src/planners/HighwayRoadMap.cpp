#include "datastructure/include/MultiBodyTree2D.h"
#include "datastructure/include/MultiBodyTree3D.h"
#include "include/HighwayRoadMap-inl.h"

template class HighwayRoadMap<MultiBodyTree2D, SuperEllipse>;
template class HighwayRoadMap<MultiBodyTree3D, SuperQuadrics>;
