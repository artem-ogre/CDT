/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/**
 * @file
 * Public API - implementation
 */

#include "CDT.h"

#include <algorithm>
#include <deque>
#include <limits>
#include <map>
#include <stdexcept>

namespace
{
std::pair<CDT::VertInd, CDT::VertInd> orderedEdgeIndices(
    const CDT::TriangleVec::const_iterator& t,
    const size_t localIndex)
{
    const CDT::VertInd edgeStart{t->vertices[localIndex]};
    const CDT::VertInd edgeEnd{t->vertices[(localIndex + 1) % 3]};
    return std::pair<CDT::VertInd, CDT::VertInd>{
        std::min(edgeStart, edgeEnd), std::max(edgeStart, edgeEnd)};
}
} // namespace

namespace CDT
{

CDT_INLINE_IF_HEADER_ONLY VerticesTriangles calculateTrianglesByVertex(
    const TriangleVec& triangles,
    const VertInd verticesSize)
{
    VerticesTriangles vertTris(verticesSize);
    for(TriInd iT = 0; iT < triangles.size(); ++iT)
    {
        const VerticesArr3& vv = triangles[iT].vertices;
        for(VerticesArr3::const_iterator v = vv.begin(); v != vv.end(); ++v)
        {
            vertTris[*v].push_back(iT);
        }
    }
    return vertTris;
}

template <typename T>
DuplicatesInfo RemoveDuplicates(std::vector<V2d<T> >& vertices)
{
    const DuplicatesInfo di = FindDuplicates<T>(
        vertices.begin(), vertices.end(), getX_V2d<T>, getY_V2d<T>);
    RemoveDuplicates(vertices, di.duplicates);
    return di;
}

CDT_INLINE_IF_HEADER_ONLY void
RemapEdges(std::vector<Edge>& edges, const std::vector<std::size_t>& mapping)
{
    RemapEdges(
        edges.begin(),
        edges.end(),
        mapping,
        edge_get_v1,
        edge_get_v2,
        edge_make);
}

template <typename T>
DuplicatesInfo RemoveDuplicatesAndRemapEdges(
    std::vector<V2d<T> >& vertices,
    std::vector<Edge>& edges)
{
    return RemoveDuplicatesAndRemapEdges<T>(
        vertices,
        getX_V2d<T>,
        getY_V2d<T>,
        edges.begin(),
        edges.end(),
        edge_get_v1,
        edge_get_v2,
        edge_is_boundary,
        edge_make);
}

CDT_INLINE_IF_HEADER_ONLY EdgeUSet
extractEdgesFromTriangles(const TriangleVec& triangles)
{
    EdgeUSet edges;
    typedef TriangleVec::const_iterator CIt;

    // Count how often the edge is used to determine if it is a boundary edge
    std::map<std::pair<VertInd, VertInd>, int> edgeCounts;
    for(CIt t = triangles.begin(); t != triangles.end(); ++t)
    {
        for(size_t localIndex = 0; localIndex < 3; ++localIndex)
        {
            edgeCounts[orderedEdgeIndices(t, localIndex)]++;
        }
    }
    for(CIt t = triangles.begin(); t != triangles.end(); ++t)
    {
        for(size_t localIndex = 0; localIndex < 3; ++localIndex)
        {
            const bool isBoundaryEdge =
                edgeCounts[orderedEdgeIndices(t, localIndex)] == 1;
            edges.insert(Edge(
                VertInd(t->vertices[localIndex]),
                VertInd(t->vertices[(localIndex + 1) % 3]),
                isBoundaryEdge));
        }
    }
    return edges;
}

CDT_INLINE_IF_HEADER_ONLY unordered_map<Edge, EdgeVec>
EdgeToPiecesMapping(const unordered_map<Edge, EdgeVec>& pieceToOriginals)
{
    unordered_map<Edge, EdgeVec> originalToPieces;
    typedef unordered_map<Edge, EdgeVec>::const_iterator Cit;
    for(Cit ptoIt = pieceToOriginals.begin(); ptoIt != pieceToOriginals.end();
        ++ptoIt)
    {
        const Edge piece = ptoIt->first;
        const EdgeVec& originals = ptoIt->second;
        for(EdgeVec::const_iterator origIt = originals.begin();
            origIt != originals.end();
            ++origIt)
        {
            originalToPieces[*origIt].push_back(piece);
        }
    }
    return originalToPieces;
}

} // namespace CDT
