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

namespace CDT
{

template class Triangulation<float>;
template class Triangulation<double>;
template struct V2d<float>;
template struct V2d<double>;
template struct Box2d<float>;
template struct Box2d<double>;
template struct Vertex<float>;
template struct Vertex<double>;

template Box2d<float> envelopBox<float>(const std::vector<V2d<float> >&);
template Box2d<double> envelopBox<double>(const std::vector<V2d<double> >&);

template DuplicatesInfo RemoveDuplicates<float>(std::vector<V2d<float> >&);
template DuplicatesInfo RemoveDuplicates<double>(std::vector<V2d<double> >&);

template DuplicatesInfo RemoveDuplicatesAndRemapEdges<float>(
    std::vector<V2d<float> >&,
    std::vector<Edge>&);
template DuplicatesInfo RemoveDuplicatesAndRemapEdges<double>(
    std::vector<V2d<double> >&,
    std::vector<Edge>&);

template std::vector<unsigned short> CalculateTriangleDepths(
    const std::vector<Vertex<float> >& vertices,
    const TriangleVec& triangles,
    const EdgeUSet& fixedEdges);
template std::vector<unsigned short> CalculateTriangleDepths(
    const std::vector<CDT::Vertex<double> >& vertices,
    const TriangleVec& triangles,
    const EdgeUSet& fixedEdges);

} // namespace CDT

#endif
