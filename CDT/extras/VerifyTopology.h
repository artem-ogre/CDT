/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#ifndef CDT_Zahj4kpHLwFgkKtcOI1i
#define CDT_Zahj4kpHLwFgkKtcOI1i

#include "CDT.h"

#include <algorithm>

namespace CDT
{

template <typename T>
inline bool verifyTopology(const CDT::Triangulation<T>& cdt)
{
    // Check if vertices' adjacent triangles contain vertex
    for(VertInd iV(0); iV < VertInd(cdt.vertices.size()); ++iV)
    {
        const Vertex<T>& v = cdt.vertices[iV];
        typedef TriIndVec::const_iterator TriIndCit;
        for(TriIndCit it = v.triangles.begin(); it != v.triangles.end(); ++it)
        {
            const array<VertInd, 3>& vv = cdt.triangles[*it].vertices;
            if(std::find(vv.begin(), vv.end(), iV) == vv.end())
                return false;
        }
    }
    // Check if triangle neighbor links are fine
    for(TriInd iT(0); iT < TriInd(cdt.triangles.size()); ++iT)
    {
        const Triangle& t = cdt.triangles[iT];
        typedef NeighborsArr3::const_iterator NCit;
        for(NCit it = t.neighbors.begin(); it != t.neighbors.end(); ++it)
        {
            if(*it == noNeighbor)
                continue;
            const array<TriInd, 3>& nn = cdt.triangles[*it].neighbors;
            if(std::find(nn.begin(), nn.end(), iT) == nn.end())
                return false;
        }
    }
    // Check if triangle's vertices have triangle as adjacent
    for(TriInd iT(0); iT < TriInd(cdt.triangles.size()); ++iT)
    {
        const Triangle& t = cdt.triangles[iT];
        typedef VerticesArr3::const_iterator VCit;
        for(VCit it = t.vertices.begin(); it != t.vertices.end(); ++it)
        {
            const std::vector<TriInd>& tt = cdt.vertices[*it].triangles;
            if(std::find(tt.begin(), tt.end(), iT) == tt.end())
                return false;
        }
    }
    return true;
}

} // namespace CDT

#endif
