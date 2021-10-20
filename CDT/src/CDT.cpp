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
#ifdef CDT_USE_BOOST
#include "LocatorBoostRTree.h"
#endif
#include "LocatorKDTree.h"
#include "LocatorNearestRandom.h"

namespace CDT
{

#ifdef CDT_USE_BOOST
template class Triangulation<float, LocatorBoostRTree<float> >;
template class Triangulation<double, LocatorBoostRTree<double> >;
#endif
template class Triangulation<float, LocatorNearestRandom<float> >;
template class Triangulation<double, LocatorNearestRandom<double> >;
template class Triangulation<float, LocatorKDTree<float> >;
template class Triangulation<double, LocatorKDTree<double> >;
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

} // namespace CDT

#endif
