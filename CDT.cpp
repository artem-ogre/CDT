#ifdef CDT_USE_AS_COMPILED_LIBRARY

#include "include/CDT.hpp"
#include "include/CDTUtils.hpp"

template class CDT::Triangulation<float>;
template class CDT::Triangulation<double>;
template struct CDT::V2d<float>;
template struct CDT::V2d<double>;
template struct CDT::Box2d<float>;
template struct CDT::Box2d<double>;
template struct CDT::Vertex<float>;
template struct CDT::Vertex<double>;

template CDT::IndexMapping
CDT::RemoveDuplicates<float>(std::vector<V2d<float> >&);
template CDT::IndexMapping
CDT::RemoveDuplicates<double>(std::vector<V2d<double> >&);

template CDT::IndexMapping CDT::RemoveDuplicatesAndRemapEdges<float>(
    std::vector<CDT::V2d<float> >&,
    std::vector<CDT::Edge>&);
template CDT::IndexMapping CDT::RemoveDuplicatesAndRemapEdges<double>(
    std::vector<CDT::V2d<double> >&,
    std::vector<CDT::Edge>&);

#endif
