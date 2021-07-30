#include "include/HighwayRoadMap-inl.h"
#include "util/include/MultiBodyTree2D.h"
#include "util/include/MultiBodyTree3D.h"

template class HighwayRoadMap<MultiBodyTree2D, SuperEllipse>;
template class HighwayRoadMap<MultiBodyTree3D, SuperQuadrics>;
