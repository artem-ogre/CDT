#ifdef CDT_USE_AS_COMPILED_LIBRARY

#include "CDT.hpp"
#include "CDTUtils.hpp"

template class CDT::Triangulation<float>;
template class CDT::Triangulation<double>;
template struct CDT::V2d<float>;
template struct CDT::V2d<double>;
template struct CDT::Box2d<float>;
template struct CDT::Box2d<double>;
template struct CDT::Vertex<float>;
template struct CDT::Vertex<double>;

#endif
