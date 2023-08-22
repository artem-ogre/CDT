/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/**
 * @file
 * Utilities and helpers - implementation
 */

#include "CDTUtils.h"

#include "predicates.h" // robust predicates: orient, in-circle

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

template <typename T>
T orient2D(const V2d<T>& p, const V2d<T>& v1, const V2d<T>& v2)
{
    return predicates::adaptive::orient2d(v1.x, v1.y, v2.x, v2.y, p.x, p.y);
}

template <typename T>
PtLineLocation::Enum locatePointLine(
    const V2d<T>& p,
    const V2d<T>& v1,
    const V2d<T>& v2,
    const T orientationTolerance)
{
    return classifyOrientation(orient2D(p, v1, v2), orientationTolerance);
}

template <typename T>
PtLineLocation::Enum
classifyOrientation(const T orientation, const T orientationTolerance)
{
    if(orientation < -orientationTolerance)
        return PtLineLocation::Right;
    if(orientation > orientationTolerance)
        return PtLineLocation::Left;
    return PtLineLocation::OnLine;
}

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
opposedTriangleInd(const VerticesArr3& vv, const VertInd iVert)
{
    assert(vv[0] == iVert || vv[1] == iVert || vv[2] == iVert);
    if(vv[0] == iVert)
        return Index(1);
    if(vv[1] == iVert)
        return Index(2);
    return Index(0);
}

CDT_INLINE_IF_HEADER_ONLY Index edgeNeighborInd(
    const VerticesArr3& vv,
    const VertInd iVedge1,
    const VertInd iVedge2)
{
    assert(vv[0] == iVedge1 || vv[1] == iVedge1 || vv[2] == iVedge1);
    assert(vv[0] == iVedge2 || vv[1] == iVedge2 || vv[2] == iVedge2);
    assert(
        (vv[0] != iVedge1 && vv[0] != iVedge2) ||
        (vv[1] != iVedge1 && vv[1] != iVedge2) ||
        (vv[2] != iVedge1 && vv[2] != iVedge2));
    /*
     *      vv[2]
     *       /\
     *  n[2]/  \n[1]
     *     /____\
     * vv[0] n[0] vv[1]
     */
    if(vv[0] == iVedge1)
    {
        if(vv[1] == iVedge2)
            return Index(0);
        return Index(2);
    }
    if(vv[0] == iVedge2)
    {
        if(vv[1] == iVedge1)
            return Index(0);
        return Index(2);
    }
    return Index(1);
}

CDT_INLINE_IF_HEADER_ONLY Index
opposedVertexInd(const NeighborsArr3& nn, const TriInd iTopo)
{
    assert(nn[0] == iTopo || nn[1] == iTopo || nn[2] == iTopo);
    if(nn[0] == iTopo)
        return Index(2);
    if(nn[1] == iTopo)
        return Index(0);
    return Index(1);
}

CDT_INLINE_IF_HEADER_ONLY Index
vertexInd(const VerticesArr3& vv, const VertInd iV)
{
    assert(vv[0] == iV || vv[1] == iV || vv[2] == iV);
    if(vv[0] == iV)
        return Index(0);
    if(vv[1] == iV)
        return Index(1);
    return Index(2);
}

CDT_INLINE_IF_HEADER_ONLY TriInd
opposedTriangle(const Triangle& tri, const VertInd iVert)
{
    return tri.neighbors[opposedTriangleInd(tri.vertices, iVert)];
}

CDT_INLINE_IF_HEADER_ONLY VertInd
opposedVertex(const Triangle& tri, const TriInd iTopo)
{
    return tri.vertices[opposedVertexInd(tri.neighbors, iTopo)];
}

/// Given triangle and an edge find neighbor sharing the edge
CDT_INLINE_IF_HEADER_ONLY TriInd
edgeNeighbor(const Triangle& tri, VertInd iVedge1, VertInd iVedge2)
{
    return tri.neighbors[edgeNeighborInd(tri.vertices, iVedge1, iVedge2)];
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
T lengthSquared(const T x, const T y)
{
    return x * x + y * y;
}

template <typename T>
T lengthSquared(const V2d<T>& v)
{
    return lengthSquared(v.x, v.y);
}

template <typename T>
T length(const V2d<T>& v)
{
    return std::sqrt(lengthSquared(v));
}

template <typename T>
T distanceSquared(const V2d<T>& a, const V2d<T>& b)
{
    return lengthSquared(b.x - a.x, b.y - a.y);
}

template <typename T>
T distance(const V2d<T>& a, const V2d<T>& b)
{
    return std::sqrt(distanceSquared(a, b));
}

bool touchesSuperTriangle(const Triangle& t)
{
    return t.vertices[0] < 3 || t.vertices[1] < 3 || t.vertices[2] < 3;
}

template <typename T>
bool isEncroachingOnEdge(
    const V2d<T>& v,
    const V2d<T>& edgeStart,
    const V2d<T>& edgeEnd)
{
    /*
     * Contains a point in its diametral circle:
     * the angle between v and edge end points is obtuse
     */
    return (edgeStart.x - v.x) * (edgeEnd.x - v.x) +
               (edgeStart.y - v.y) * (edgeEnd.y - v.y) <=
           T(0);
}

template <typename T>
V2d<T> circumcenter(V2d<T> a, V2d<T> b, V2d<T> c)
{
    const T denom = T(2) * orient2D(a, b, c);
    assert(denom != T(0));
    a.x -= c.x, a.y -= c.y;
    b.x -= c.x, b.y -= c.y;
    const T aLenSq = lengthSquared(a), bLenSq = lengthSquared(b);
    c.x += (b.y * aLenSq - a.y * bLenSq) / denom;
    c.y += (a.x * bLenSq - b.x * aLenSq) / denom;
    return c;
}

template <typename T>
T doubledArea(const V2d<T>& a, const V2d<T>& b, const V2d<T>& c)
{
    return std::abs(orient2D(a, b, c));
}

template <typename T>
T area(const V2d<T>& a, const V2d<T>& b, const V2d<T>& c)
{
    return doubledArea(a, b, c) / T(2);
}

template <typename T>
T sineOfSmallestAngle(const V2d<T>& a, const V2d<T>& b, const V2d<T>& c)
{
    // find sides of the smallest angle using law of sines:
    T sideA = distance(a, b), sideB = distance(b, c);
    if(sideA > sideB)
        std::swap(sideA, sideB);
    sideA = std::max(sideA, distance(a, c));
    return (doubledArea(a, b, c) / sideA) / sideB;
}

template <typename T>
T smallestAngle(const V2d<T>& a, const V2d<T>& b, const V2d<T>& c)
{
    const T angleSine = sineOfSmallestAngle(a, b, c);
    assert(angleSine >= -1 && angleSine <= 1);
    return std::asin(angleSine);
}

bool touchesSuperTriangle(const Triangle& t)
{
    return t.vertices[0] < 3 || t.vertices[1] < 3 || t.vertices[2] < 3;
}

} // namespace CDT
