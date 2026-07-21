/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/**
 * @file
 * Instantiates the public API so that it is compiled and not merely parsed.
 *
 * Compiled directly by the "C++ standards" CI job (not through CMake, which
 * builds the Catch2 tests as c++17) to check that CDT builds with every
 * supported standard. Keep this file c++98-compatible.
 */

#include <CDT.h>
#include <InitializeWithGrid.h>
#include <VerifyTopology.h>

#include <cstdlib>
#include <vector>

namespace
{

template <typename T>
bool checkCoordinateType()
{
    std::vector<CDT::V2d<T> > vv;
    vv.push_back(CDT::V2d<T>(T(0), T(0)));
    vv.push_back(CDT::V2d<T>(T(1), T(0)));
    vv.push_back(CDT::V2d<T>(T(1), T(1)));
    vv.push_back(CDT::V2d<T>(T(0), T(1)));
    vv.push_back(CDT::V2d<T>(T(0), T(1))); // duplicate

    std::vector<CDT::Edge> ee;
    ee.push_back(CDT::Edge(CDT::VertInd(0), CDT::VertInd(2)));

    const CDT::DuplicatesInfo di = CDT::RemoveDuplicatesAndRemapEdges(vv, ee);
    if(di.duplicates.size() != 1)
        return false;

    CDT::Triangulation<T> cdt;
    cdt.insertVertices(vv);
    cdt.insertEdges(ee);
    if(!CDT::verifyTopology(cdt))
        return false;
    if(!CDT::eachVertexHasNeighborTriangle(cdt))
        return false;
    cdt.eraseSuperTriangle();
    if(cdt.triangles.empty())
        return false;

    CDT::Triangulation<T> grid;
    CDT::initializeWithRegularGrid(T(0), T(1), T(0), T(1), 2, 2, grid);
    if(!CDT::verifyTopology(grid))
        return false;

    return true;
}

} // namespace

int main()
{
    if(!checkCoordinateType<float>())
        return EXIT_FAILURE;
    if(!checkCoordinateType<double>())
        return EXIT_FAILURE;
    return EXIT_SUCCESS;
}
