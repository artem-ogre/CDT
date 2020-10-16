#ifdef CDT_USE_AS_COMPILED_LIBRARY

#include <CDT.hpp>
#include <CDTUtils.hpp>

template class CDT::Triangulation<float>;
template class CDT::Triangulation<double>;
template struct CDT::V2d<float>;
template struct CDT::V2d<double>;
template struct CDT::Box2d<float>;
template struct CDT::Box2d<double>;
template struct CDT::Vertex<float>;
template struct CDT::Vertex<double>;

template std::vector<std::size_t>
CDT::RemoveDuplicates<float>(std::vector<V2d<float> >&);
template std::vector<std::size_t>
CDT::RemoveDuplicates<double>(std::vector<V2d<double> >&);

template std::vector<std::size_t> CDT::RemoveDuplicatesAndRemapEdges<float>(
    std::vector<CDT::V2d<float> >&,
    std::vector<CDT::Edge>&);
template std::vector<std::size_t> CDT::RemoveDuplicatesAndRemapEdges<double>(
    std::vector<CDT::V2d<double> >&,
    std::vector<CDT::Edge>&);

#endif
