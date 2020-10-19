/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

 /**
 * @file
 * Explicit template instantiations for `float` and `double` coordinate types.
 *
 * Explicit instantiations are used when consuming as a compiled library.
 * In almost all of the cases 'float' and 'double' are sufficient.
 * Feel free to extend this with custom data types if needed.
 */

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
