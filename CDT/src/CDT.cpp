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

#include "CDTUtils.hpp"
#include "CDT.hpp"
#include "InitializeWithGrid.h"
#include "VerifyTopology.h"

namespace CDT
{

template class Triangulation<float>;
template class Triangulation<double>;
template struct V2d<float>;
template struct V2d<double>;
template struct Box2d<float>;
template struct Box2d<double>;

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

template bool verifyTopology<float>(const CDT::Triangulation<float>&);
template bool verifyTopology<double>(const CDT::Triangulation<double>&);

template void initializeWithRegularGrid<float>(
    float,
    float,
    float,
    float,
    std::size_t,
    std::size_t,
    Triangulation<float>&);
template void initializeWithRegularGrid<double>(
    double,
    double,
    double,
    double,
    std::size_t,
    std::size_t,
    Triangulation<double>&);

} // namespace CDT

#endif
