/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/**
 * @file
 * Utilities and helpers - implementation
 */

#include "CDTUtils.h"

//! @{ Doxygen suppression
#include "predicates.h" // robust predicates: orient, in-circle
//! @}

#include <stdexcept>

namespace CDT
{

//*****************************************************************************
// V2d
//*****************************************************************************
template <typename T>
V2d<T> V2d<T>::make(const T x, const T y)
{
    V2d<T> out = {x, y};
    return out;
}

//*****************************************************************************
// Box2d
//*****************************************************************************
template <typename T>
Box2d<T> envelopBox(const std::vector<V2d<T> >& vertices)
{
    return envelopBox<T>(
        vertices.begin(), vertices.end(), getX_V2d<T>, getY_V2d<T>);
}

//*****************************************************************************
// Edge
//*****************************************************************************
CDT_INLINE_IF_HEADER_ONLY Edge::Edge(VertInd iV1, VertInd iV2)
    : m_vertices(
          iV1 < iV2 ? std::make_pair(iV1, iV2) : std::make_pair(iV2, iV1))
{}

CDT_INLINE_IF_HEADER_ONLY bool Edge::operator==(const Edge& other) const
{
    return m_vertices == other.m_vertices;
}

CDT_INLINE_IF_HEADER_ONLY bool Edge::operator!=(const Edge& other) const
{
    return !(this->operator==(other));
}

CDT_INLINE_IF_HEADER_ONLY VertInd Edge::v1() const
{
    return m_vertices.first;
}

CDT_INLINE_IF_HEADER_ONLY VertInd Edge::v2() const
{
    return m_vertices.second;
}

CDT_INLINE_IF_HEADER_ONLY const std::pair<VertInd, VertInd>& Edge::verts() const
{
    return m_vertices;
}

//*****************************************************************************
// Utility functions
//*****************************************************************************
CDT_INLINE_IF_HEADER_ONLY Index ccw(Index i)
{
    return Index((i + 1) % 3);
}

CDT_INLINE_IF_HEADER_ONLY Index cw(Index i)
{
    return Index((i + 2) % 3);
}

CDT_INLINE_IF_HEADER_ONLY bool isOnEdge(const PtTriLocation::Enum location)
{
    return location > PtTriLocation::Outside;
}

CDT_INLINE_IF_HEADER_ONLY Index edgeNeighbor(const PtTriLocation::Enum location)
{
    assert(location >= PtTriLocation::OnEdge1);
    return static_cast<Index>(location - PtTriLocation::OnEdge1);
}

/// Check if point lies to the left of, to the right of, or on a line
template <typename T>
PtLineLocation::Enum
locatePointLine(const V2d<T>& p, const V2d<T>& v1, const V2d<T>& v2)
{
    using namespace predicates::adaptive;
    const T orientation = orient2d(v1.x, v1.y, v2.x, v2.y, p.x, p.y);
    if(orientation < T(0))
        return PtLineLocation::Right;
    if(orientation == T(0))
        return PtLineLocation::OnLine;
    return PtLineLocation::Left;
}

/// Check if point a lies inside of, outside of, or on an edge of a triangle
template <typename T>
PtTriLocation::Enum locatePointTriangle(
    const V2d<T>& p,
    const V2d<T>& v1,
    const V2d<T>& v2,
    const V2d<T>& v3)
{
    using namespace predicates::adaptive;
    PtTriLocation::Enum result = PtTriLocation::Inside;
    PtLineLocation::Enum edgeCheck = locatePointLine(p, v1, v2);
    if(edgeCheck == PtLineLocation::Right)
        return PtTriLocation::Outside;
    if(edgeCheck == PtLineLocation::OnLine)
        result = PtTriLocation::OnEdge1;
    edgeCheck = locatePointLine(p, v2, v3);
    if(edgeCheck == PtLineLocation::Right)
        return PtTriLocation::Outside;
    if(edgeCheck == PtLineLocation::OnLine)
        result = PtTriLocation::OnEdge2;
    edgeCheck = locatePointLine(p, v3, v1);
    if(edgeCheck == PtLineLocation::Right)
        return PtTriLocation::Outside;
    if(edgeCheck == PtLineLocation::OnLine)
        result = PtTriLocation::OnEdge3;
    return result;
}

CDT_INLINE_IF_HEADER_ONLY Index opoNbr(const Index vertIndex)
{
    if(vertIndex == Index(0))
        return Index(1);
    if(vertIndex == Index(1))
        return Index(2);
    if(vertIndex == Index(2))
        return Index(0);
    throw std::runtime_error("Invalid vertex index");
}

CDT_INLINE_IF_HEADER_ONLY Index opoVrt(const Index neighborIndex)
{
    if(neighborIndex == Index(0))
        return Index(2);
    if(neighborIndex == Index(1))
        return Index(0);
    if(neighborIndex == Index(2))
        return Index(1);
    throw std::runtime_error("Invalid neighbor index");
}

CDT_INLINE_IF_HEADER_ONLY Index
opposedTriangleInd(const Triangle& tri, const VertInd iVert)
{
    for(Index vi = Index(0); vi < Index(3); ++vi)
        if(iVert == tri.vertices[vi])
            return opoNbr(vi);
    throw std::runtime_error("Could not find opposed triangle index");
}

CDT_INLINE_IF_HEADER_ONLY Index opposedTriangleInd(
    const Triangle& tri,
    const VertInd iVedge1,
    const VertInd iVedge2)
{
    for(Index vi = Index(0); vi < Index(3); ++vi)
    {
        const VertInd iVert = tri.vertices[vi];
        if(iVert != iVedge1 && iVert != iVedge2)
            return opoNbr(vi);
    }
    throw std::runtime_error("Could not find opposed-to-edge triangle index");
}

CDT_INLINE_IF_HEADER_ONLY Index
opposedVertexInd(const Triangle& tri, const TriInd iTopo)
{
    for(Index ni = Index(0); ni < Index(3); ++ni)
        if(iTopo == tri.neighbors[ni])
            return opoVrt(ni);
    throw std::runtime_error("Could not find opposed vertex index");
}

CDT_INLINE_IF_HEADER_ONLY Index
neighborInd(const Triangle& tri, const TriInd iTnbr)
{
    for(Index ni = Index(0); ni < Index(3); ++ni)
        if(iTnbr == tri.neighbors[ni])
            return ni;
    throw std::runtime_error("Could not find neighbor triangle index");
}

CDT_INLINE_IF_HEADER_ONLY Index vertexInd(const Triangle& tri, const VertInd iV)
{
    for(Index i = Index(0); i < Index(3); ++i)
        if(iV == tri.vertices[i])
            return i;
    throw std::runtime_error("Could not find vertex index in triangle");
}

CDT_INLINE_IF_HEADER_ONLY TriInd
opposedTriangle(const Triangle& tri, const VertInd iVert)
{
    return tri.neighbors[opposedTriangleInd(tri, iVert)];
}

CDT_INLINE_IF_HEADER_ONLY VertInd
opposedVertex(const Triangle& tri, const TriInd iTopo)
{
    return tri.vertices[opposedVertexInd(tri, iTopo)];
}

template <typename T>
bool isInCircumcircle(
    const V2d<T>& p,
    const V2d<T>& v1,
    const V2d<T>& v2,
    const V2d<T>& v3)
{
    using namespace predicates::adaptive;
    return incircle(v1.x, v1.y, v2.x, v2.y, v3.x, v3.y, p.x, p.y) > T(0);
}

CDT_INLINE_IF_HEADER_ONLY
bool verticesShareEdge(const TriIndVec& aTris, const TriIndVec& bTris)
{
    for(TriIndVec::const_iterator it = aTris.begin(); it != aTris.end(); ++it)
        if(std::find(bTris.begin(), bTris.end(), *it) != bTris.end())
            return true;
    return false;
}

template <typename T>
T distanceSquared(const T ax, const T ay, const T bx, const T by)
{
    const T dx = bx - ax;
    const T dy = by - ay;
    return dx * dx + dy * dy;
}

template <typename T>
T distance(const T ax, const T ay, const T bx, const T by)
{
    return std::sqrt(distanceSquared(ax, ay, bx, by));
}

template <typename T>
T distance(const V2d<T>& a, const V2d<T>& b)
{
    return distance(a.x, a.y, b.x, b.y);
}

template <typename T>
T distanceSquared(const V2d<T>& a, const V2d<T>& b)
{
    return distanceSquared(a.x, a.y, b.x, b.y);
}

} // namespace CDT
