/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/**
 * @file
 * Public API - implementation
 */

#include "CDT.h"

#include "CDTUtils.h"
#include <algorithm>
#include <deque>
#include <stdexcept>

namespace CDT
{

typedef std::deque<TriInd> TriDeque;

namespace detail
{

/// Needed for c++03 compatibility (no uniform initialization available)
template <typename T>
array<T, 3> arr3(const T& v0, const T& v1, const T& v2)
{
    const array<T, 3> out = {v0, v1, v2};
    return out;
}

} // namespace detail

template <typename T, typename TNearPointLocator>
Triangulation<T, TNearPointLocator>::Triangulation()
    : m_nTargetVerts(0)
    , m_superGeomType(SuperGeometryType::SuperTriangle)
    , m_vertexInsertionOrder(VertexInsertionOrder::Randomized)
{}

template <typename T, typename TNearPointLocator>
Triangulation<T, TNearPointLocator>::Triangulation(
    const VertexInsertionOrder::Enum vertexInsertionOrder)
    : m_nTargetVerts(0)
    , m_superGeomType(SuperGeometryType::SuperTriangle)
    , m_vertexInsertionOrder(vertexInsertionOrder)
{}

template <typename T, typename TNearPointLocator>
Triangulation<T, TNearPointLocator>::Triangulation(
    VertexInsertionOrder::Enum vertexInsertionOrder,
    const TNearPointLocator& nearPtLocator)
    : m_nTargetVerts(0)
    , m_nearPtLocator(nearPtLocator)
    , m_superGeomType(SuperGeometryType::SuperTriangle)
    , m_vertexInsertionOrder(vertexInsertionOrder)
{}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::changeNeighbor(
    const TriInd iT,
    const VertInd iVedge1,
    const VertInd iVedge2,
    const TriInd newNeighbor)
{
    Triangle& t = triangles[iT];
    t.neighbors[opposedTriangleInd(t, iVedge1, iVedge2)] = newNeighbor;
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::eraseDummies()
{
    if(m_dummyTris.empty())
        return;
    const TriIndUSet dummySet(m_dummyTris.begin(), m_dummyTris.end());
    TriIndUMap triIndMap;
    triIndMap[noNeighbor] = noNeighbor;
    for(TriInd iT(0), iTnew(0); iT < TriInd(triangles.size()); ++iT)
    {
        if(dummySet.count(iT))
            continue;
        triIndMap[iT] = iTnew;
        triangles[iTnew] = triangles[iT];
        iTnew++;
    }
    triangles.erase(triangles.end() - dummySet.size(), triangles.end());

    // remap adjacent triangle indices for vertices
    typedef typename VerticesTriangles::iterator VertTrisIt;
    for(VertTrisIt vTris = vertTris.begin(); vTris != vertTris.end(); ++vTris)
    {
        for(TriIndVec::iterator iT = vTris->begin(); iT != vTris->end(); ++iT)
            *iT = triIndMap[*iT];
    }
    // remap neighbor indices for triangles
    for(TriangleVec::iterator t = triangles.begin(); t != triangles.end(); ++t)
    {
        NeighborsArr3& nn = t->neighbors;
        for(NeighborsArr3::iterator iN = nn.begin(); iN != nn.end(); ++iN)
            *iN = triIndMap[*iN];
    }
    // clear dummy triangles
    m_dummyTris = std::vector<TriInd>();
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::eraseSuperTriangleVertices()
{
    if(m_superGeomType != SuperGeometryType::SuperTriangle)
        return;
    for(TriangleVec::iterator t = triangles.begin(); t != triangles.end(); ++t)
        for(Index i(0); i < Index(3); ++i)
            t->vertices[i] -= 3;

    EdgeUSet updatedFixedEdges;
    typedef CDT::EdgeUSet::const_iterator EdgeCit;
    for(EdgeCit e = fixedEdges.begin(); e != fixedEdges.end(); ++e)
    {
        updatedFixedEdges.insert(
            Edge(VertInd(e->v1() - 3), VertInd(e->v2() - 3)));
    }
    fixedEdges = updatedFixedEdges;

    vertices = std::vector<V2d<T> >(vertices.begin() + 3, vertices.end());
    vertTris = VerticesTriangles(vertTris.begin() + 3, vertTris.end());
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::eraseSuperTriangle()
{
    if(m_superGeomType != SuperGeometryType::SuperTriangle)
        return;
    // make dummy triangles adjacent to super-triangle's vertices
    for(TriInd iT(0); iT < TriInd(triangles.size()); ++iT)
    {
        Triangle& t = triangles[iT];
        if(t.vertices[0] < 3 || t.vertices[1] < 3 || t.vertices[2] < 3)
            makeDummy(iT);
    }
    eraseDummies();
    eraseSuperTriangleVertices();
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::eraseOuterTriangles()
{
    // make dummy triangles adjacent to super-triangle's vertices
    const std::stack<TriInd> seed(std::deque<TriInd>(1, vertTris[0].front()));
    const TriIndUSet toErase = growToBoundary(seed);
    eraseTrianglesAtIndices(toErase.begin(), toErase.end());
    eraseSuperTriangleVertices();
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::eraseOuterTrianglesAndHoles()
{
    const std::vector<LayerDepth> triDepths = CalculateTriangleDepths(
        vertTris[0].front(), triangles, fixedEdges, overlapCount);

    TriIndVec toErase;
    toErase.reserve(triangles.size());
    for(std::size_t iT = 0; iT != triangles.size(); ++iT)
    {
        if(triDepths[iT] % 2 == 0)
            toErase.push_back(iT);
    }

    eraseTrianglesAtIndices(toErase.begin(), toErase.end());
    eraseSuperTriangleVertices();
}

template <typename T, typename TNearPointLocator>
template <typename TriIndexIter>
void Triangulation<T, TNearPointLocator>::eraseTrianglesAtIndices(
    TriIndexIter first,
    TriIndexIter last)
{
    for(; first != last; ++first)
        makeDummy(*first);
    eraseDummies();
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::initializedWithCustomSuperGeometry()
{
    for(std::size_t i = 0; i < vertices.size(); ++i)
    {
        m_nearPtLocator.addPoint(VertInd(i), vertices);
    }
    m_nTargetVerts = vertices.size();
    m_superGeomType = SuperGeometryType::Custom;
}

template <typename T, typename TNearPointLocator>
TriIndUSet Triangulation<T, TNearPointLocator>::growToBoundary(
    std::stack<TriInd> seeds) const
{
    TriIndUSet traversed;
    while(!seeds.empty())
    {
        const TriInd iT = seeds.top();
        seeds.pop();
        traversed.insert(iT);
        const Triangle& t = triangles[iT];
        for(Index i(0); i < Index(3); ++i)
        {
            const Edge opEdge(t.vertices[ccw(i)], t.vertices[cw(i)]);
            if(fixedEdges.count(opEdge))
                continue;
            const TriInd iN = t.neighbors[opoNbr(i)];
            if(iN != noNeighbor && traversed.count(iN) == 0)
                seeds.push(iN);
        }
    }
    return traversed;
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::makeDummy(const TriInd iT)
{
    const Triangle& t = triangles[iT];

    typedef VerticesArr3::const_iterator VCit;
    for(VCit iV = t.vertices.begin(); iV != t.vertices.end(); ++iV)
        removeAdjacentTriangle(*iV, iT);

    typedef NeighborsArr3::const_iterator NCit;
    for(NCit iTn = t.neighbors.begin(); iTn != t.neighbors.end(); ++iTn)
        changeNeighbor(*iTn, iT, noNeighbor);

    m_dummyTris.push_back(iT);
}

template <typename T, typename TNearPointLocator>
TriInd Triangulation<T, TNearPointLocator>::addTriangle(const Triangle& t)
{
    if(m_dummyTris.empty())
    {
        triangles.push_back(t);
        return TriInd(triangles.size() - 1);
    }
    const TriInd nxtDummy = m_dummyTris.back();
    m_dummyTris.pop_back();
    triangles[nxtDummy] = t;
    return nxtDummy;
}

template <typename T, typename TNearPointLocator>
TriInd Triangulation<T, TNearPointLocator>::addTriangle()
{
    if(m_dummyTris.empty())
    {
        const Triangle dummy = {
            {noVertex, noVertex, noVertex},
            {noNeighbor, noNeighbor, noNeighbor}};
        triangles.push_back(dummy);
        return TriInd(triangles.size() - 1);
    }
    const TriInd nxtDummy = m_dummyTris.back();
    m_dummyTris.pop_back();
    return nxtDummy;
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::insertEdges(
    const std::vector<Edge>& edges)
{
    insertEdges(edges.begin(), edges.end(), edge_get_v1, edge_get_v2);
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::fixEdge(const Edge& edge)
{
    if(!fixedEdges.insert(edge).second)
    {
        ++overlapCount[edge]; // if edge is already fixed bump a counter
    }
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::insertEdge(Edge edge)
{
    const VertInd iA = edge.v1();
    VertInd iB = edge.v2();
    if(iA == iB) // edge connects a vertex to itself
        return;
    const TriIndVec& aTris = vertTris[iA];
    const TriIndVec& bTris = vertTris[iB];
    const V2d<T>& a = vertices[iA];
    const V2d<T>& b = vertices[iB];
    if(verticesShareEdge(aTris, bTris))
    {
        fixEdge(Edge(iA, iB));
        return;
    }
    TriInd iT;
    VertInd iVleft, iVright;
    tie(iT, iVleft, iVright) = intersectedTriangle(iA, aTris, a, b);
    // if one of the triangle vertices is on the edge, move edge start
    if(iT == noNeighbor)
    {
        fixEdge(Edge(iA, iVleft));
        return insertEdge(Edge(iVleft, iB));
    }
    std::vector<TriInd> intersected(1, iT);
    std::vector<VertInd> ptsLeft(1, iVleft);
    std::vector<VertInd> ptsRight(1, iVright);
    VertInd iV = iA;
    Triangle t = triangles[iT];
    const VerticesArr3& tverts = t.vertices;
    while(std::find(tverts.begin(), tverts.end(), iB) == tverts.end())
    {
        const TriInd iTopo = opposedTriangle(t, iV);
        const Triangle& tOpo = triangles[iTopo];
        const VertInd iVopo = opposedVertex(tOpo, iT);
        const V2d<T> vOpo = vertices[iVopo];

        intersected.push_back(iTopo);
        iT = iTopo;
        t = triangles[iT];

        const PtLineLocation::Enum loc = locatePointLine(vOpo, a, b);
        if(loc == PtLineLocation::Left)
        {
            ptsLeft.push_back(iVopo);
            iV = iVleft;
            iVleft = iVopo;
        }
        else if(loc == PtLineLocation::Right)
        {
            ptsRight.push_back(iVopo);
            iV = iVright;
            iVright = iVopo;
        }
        else // encountered point on the edge
            iB = iVopo;
    }
    // Remove intersected triangles
    typedef std::vector<TriInd>::const_iterator TriIndCit;
    for(TriIndCit it = intersected.begin(); it != intersected.end(); ++it)
        makeDummy(*it);
    // Triangulate pseudo-polygons on both sides
    const TriInd iTleft = triangulatePseudopolygon(iA, iB, ptsLeft);
    std::reverse(ptsRight.begin(), ptsRight.end());
    const TriInd iTright = triangulatePseudopolygon(iB, iA, ptsRight);
    changeNeighbor(iTleft, noNeighbor, iTright);
    changeNeighbor(iTright, noNeighbor, iTleft);
    // add fixed edge
    fixEdge(Edge(iA, iB));
    if(iB != edge.v2()) // encountered point on the edge
        return insertEdge(Edge(iB, edge.v2()));
}

/*!
 * Returns:
 *  - intersected triangle index
 *  - index of point on the left of the line
 *  - index of point on the right of the line
 * If left point is right on the line: no triangle is intersected:
 *  - triangle index is no-neighbor (invalid)
 *  - index of point on the line
 *  - index of point on the right of the line
 */
template <typename T, typename TNearPointLocator>
tuple<TriInd, VertInd, VertInd>
Triangulation<T, TNearPointLocator>::intersectedTriangle(
    const VertInd iA,
    const std::vector<TriInd>& candidates,
    const V2d<T>& a,
    const V2d<T>& b) const
{
    typedef std::vector<TriInd>::const_iterator TriIndCit;
    for(TriIndCit it = candidates.begin(); it != candidates.end(); ++it)
    {
        const TriInd iT = *it;
        const Triangle t = triangles[iT];
        const Index i = vertexInd(t, iA);
        const VertInd iP1 = t.vertices[cw(i)];
        const VertInd iP2 = t.vertices[ccw(i)];
        const PtLineLocation::Enum locP1 = locatePointLine(vertices[iP1], a, b);
        const PtLineLocation::Enum locP2 = locatePointLine(vertices[iP2], a, b);
        if(locP2 == PtLineLocation::Right)
        {
            if(locP1 == PtLineLocation::OnLine)
                return make_tuple(noNeighbor, iP1, iP2);
            if(locP1 == PtLineLocation::Left)
                return make_tuple(iT, iP1, iP2);
        }
    }
    throw std::runtime_error("Could not find vertex triangle intersected by "
                             "edge. Note: can be caused by duplicate points.");
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::addSuperTriangle(const Box2d<T>& box)
{
    m_nTargetVerts = 3;
    m_superGeomType = SuperGeometryType::SuperTriangle;

    const V2d<T> center = {
        (box.min.x + box.max.x) / T(2), (box.min.y + box.max.y) / T(2)};
    const T w = box.max.x - box.min.x;
    const T h = box.max.y - box.min.y;
    T r = std::sqrt(w * w + h * h) / T(2); // incircle radius
    r *= T(1.1);
    const T R = T(2) * r;                        // excircle radius
    const T shiftX = R * std::sqrt(T(3)) / T(2); // R * cos(30 deg)
    const V2d<T> posV1 = {center.x - shiftX, center.y - r};
    const V2d<T> posV2 = {center.x + shiftX, center.y - r};
    const V2d<T> posV3 = {center.x, center.y + R};
    addNewVertex(posV1, TriIndVec(1, TriInd(0)));
    addNewVertex(posV2, TriIndVec(1, TriInd(0)));
    addNewVertex(posV3, TriIndVec(1, TriInd(0)));
    const Triangle superTri = {
        {VertInd(0), VertInd(1), VertInd(2)},
        {noNeighbor, noNeighbor, noNeighbor}};
    addTriangle(superTri);

    m_nearPtLocator.addPoint(VertInd(0), vertices);
    m_nearPtLocator.addPoint(VertInd(1), vertices);
    m_nearPtLocator.addPoint(VertInd(2), vertices);
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::addNewVertex(
    const V2d<T>& pos,
    const TriIndVec& tris)
{
    vertices.push_back(pos);
    vertTris.push_back(tris);
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::insertVertex(const VertInd iVert)
{
    const V2d<T>& v = vertices[iVert];
    array<TriInd, 2> trisAt = walkingSearchTrianglesAt(v);
    std::stack<TriInd> triStack =
        trisAt[1] == noNeighbor
            ? insertPointInTriangle(iVert, trisAt[0])
            : insertPointOnEdge(iVert, trisAt[0], trisAt[1]);
    while(!triStack.empty())
    {
        const TriInd iT = triStack.top();
        triStack.pop();

        const Triangle& t = triangles[iT];
        const TriInd iTopo = opposedTriangle(t, iVert);
        if(iTopo == noNeighbor)
            continue;
        if(isFlipNeeded(v, iT, iTopo, iVert))
        {
            flipEdge(iT, iTopo);
            triStack.push(iT);
            triStack.push(iTopo);
        }
    }

    m_nearPtLocator.addPoint(iVert, vertices);
}

/*!
 * Handles super-triangle vertices.
 * Super-tri points are not infinitely far and influence the input points
 * Three cases are possible:
 *  1.  If one of the opposed vertices is super-tri: no flip needed
 *  2.  One of the shared vertices is super-tri:
 *      check if on point is same side of line formed by non-super-tri vertices
 *      as the non-super-tri shared vertex
 *  3.  None of the vertices are super-tri: normal circumcircle test
 */
template <typename T, typename TNearPointLocator>
bool Triangulation<T, TNearPointLocator>::isFlipNeeded(
    const V2d<T>& v,
    const TriInd iT,
    const TriInd iTopo,
    const VertInd iV) const
{
    /*
     *                       v3         original edge: (v1, v3)
     *                      /|\   flip-candidate edge: (v,  v2)
     *                    /  |  \
     *                  /    |    \
     *                /      |      \
     * new vertex--> v       |       v2
     *                \      |      /
     *                  \    |    /
     *                    \  |  /
     *                      \|/
     *                       v1
     */
    const Triangle& tOpo = triangles[iTopo];
    const Index i = opposedVertexInd(tOpo, iT);
    const VertInd iV2 = tOpo.vertices[i];
    const VertInd iV1 = tOpo.vertices[cw(i)];
    const VertInd iV3 = tOpo.vertices[ccw(i)];
    const V2d<T>& v1 = vertices[iV1];
    const V2d<T>& v2 = vertices[iV2];
    const V2d<T>& v3 = vertices[iV3];
    if(m_superGeomType == SuperGeometryType::SuperTriangle)
    {
        // If flip-candidate edge touches super-triangle in-circumference test
        // has to be replaced with orient2d test against the line formed by two
        // non-artificial vertices (that don't belong to super-triangle)
        if(iV < 3) // flip-candidate edge touches super-triangle
        {
            // does original edge also touch super-triangle?
            if(iV1 < 3)
                return locatePointLine(v1, v2, v3) ==
                       locatePointLine(v, v2, v3);
            if(iV3 < 3)
                return locatePointLine(v3, v1, v2) ==
                       locatePointLine(v, v1, v2);
            return false; // original edge does not touch super-triangle
        }
        if(iV2 < 3) // flip-candidate edge touches super-triangle
        {
            // does original edge also touch super-triangle?
            if(iV1 < 3)
                return locatePointLine(v1, v, v3) == locatePointLine(v2, v, v3);
            if(iV3 < 3)
                return locatePointLine(v3, v1, v) == locatePointLine(v2, v1, v);
            return false; // original edge does not touch super-triangle
        }
        // flip-candidate edge does not touch super-triangle
        if(iV1 < 3)
            return locatePointLine(v1, v2, v3) == locatePointLine(v, v2, v3);
        if(iV3 < 3)
            return locatePointLine(v3, v1, v2) == locatePointLine(v, v1, v2);
    }
    return isInCircumcircle(v, v1, v2, v3);
}

/* Insert point into triangle: split into 3 triangles:
 *  - create 2 new triangles
 *  - re-use old triangle for the 3rd
 *                      v3
 *                    / | \
 *                   /  |  \ <-- original triangle (t)
 *                  /   |   \
 *              n3 /    |    \ n2
 *                /newT2|newT1\
 *               /      v      \
 *              /    __/ \__    \
 *             /  __/       \__  \
 *            / _/      t'     \_ \
 *          v1 ___________________ v2
 *                     n1
 */
template <typename T, typename TNearPointLocator>
std::stack<TriInd> Triangulation<T, TNearPointLocator>::insertPointInTriangle(
    const VertInd v,
    const TriInd iT)
{
    const TriInd iNewT1 = addTriangle();
    const TriInd iNewT2 = addTriangle();

    Triangle& t = triangles[iT];
    const array<VertInd, 3> vv = t.vertices;
    const array<TriInd, 3> nn = t.neighbors;
    const VertInd v1 = vv[0], v2 = vv[1], v3 = vv[2];
    const TriInd n1 = nn[0], n2 = nn[1], n3 = nn[2];
    // make two new triangles and convert current triangle to 3rd new triangle
    using detail::arr3;
    triangles[iNewT1] = Triangle::make(arr3(v2, v3, v), arr3(n2, iNewT2, iT));
    triangles[iNewT2] = Triangle::make(arr3(v3, v1, v), arr3(n3, iT, iNewT1));
    t = Triangle::make(arr3(v1, v2, v), arr3(n1, iNewT1, iNewT2));
    // make and add a new vertex
    addAdjacentTriangles(v, iT, iNewT1, iNewT2);
    // adjust lists of adjacent triangles for v1, v2, v3
    addAdjacentTriangle(v1, iNewT2);
    addAdjacentTriangle(v2, iNewT1);
    removeAdjacentTriangle(v3, iT);
    addAdjacentTriangle(v3, iNewT1);
    addAdjacentTriangle(v3, iNewT2);
    // change triangle neighbor's neighbors to new triangles
    changeNeighbor(n2, iT, iNewT1);
    changeNeighbor(n3, iT, iNewT2);
    // return newly added triangles
    std::stack<TriInd> newTriangles;
    newTriangles.push(iT);
    newTriangles.push(iNewT1);
    newTriangles.push(iNewT2);
    return newTriangles;
}

/* Inserting a point on the edge between two triangles
 *    T1 (top)        v1
 *                   /|\
 *              n1 /  |  \ n4
 *               /    |    \
 *             /  T1' | Tnew1\
 *           v2-------v-------v4
 *             \ Tnew2| T2'  /
 *               \    |    /
 *              n2 \  |  / n3
 *                   \|/
 *   T2 (bottom)      v3
 */
template <typename T, typename TNearPointLocator>
std::stack<TriInd> Triangulation<T, TNearPointLocator>::insertPointOnEdge(
    const VertInd v,
    const TriInd iT1,
    const TriInd iT2)
{
    const TriInd iTnew1 = addTriangle();
    const TriInd iTnew2 = addTriangle();

    Triangle& t1 = triangles[iT1];
    Triangle& t2 = triangles[iT2];
    Index i = opposedVertexInd(t1, iT2);
    const VertInd v1 = t1.vertices[i];
    const VertInd v2 = t1.vertices[ccw(i)];
    const TriInd n1 = t1.neighbors[i];
    const TriInd n4 = t1.neighbors[cw(i)];
    i = opposedVertexInd(t2, iT1);
    const VertInd v3 = t2.vertices[i];
    const VertInd v4 = t2.vertices[ccw(i)];
    const TriInd n3 = t2.neighbors[i];
    const TriInd n2 = t2.neighbors[cw(i)];
    // add new triangles and change existing ones
    using detail::arr3;
    t1 = Triangle::make(arr3(v1, v2, v), arr3(n1, iTnew2, iTnew1));
    t2 = Triangle::make(arr3(v3, v4, v), arr3(n3, iTnew1, iTnew2));
    triangles[iTnew1] = Triangle::make(arr3(v1, v, v4), arr3(iT1, iT2, n4));
    triangles[iTnew2] = Triangle::make(arr3(v3, v, v2), arr3(iT2, iT1, n2));
    // make and add new vertex
    addAdjacentTriangles(v, iT1, iTnew2, iT2, iTnew1);
    // adjust neighboring triangles and vertices
    changeNeighbor(n4, iT1, iTnew1);
    changeNeighbor(n2, iT2, iTnew2);
    addAdjacentTriangle(v1, iTnew1);
    addAdjacentTriangle(v3, iTnew2);
    removeAdjacentTriangle(v2, iT2);
    addAdjacentTriangle(v2, iTnew2);
    removeAdjacentTriangle(v4, iT1);
    addAdjacentTriangle(v4, iTnew1);
    // return newly added triangles
    std::stack<TriInd> newTriangles;
    newTriangles.push(iT1);
    newTriangles.push(iTnew2);
    newTriangles.push(iT2);
    newTriangles.push(iTnew1);
    return newTriangles;
}

template <typename T, typename TNearPointLocator>
array<TriInd, 2>
Triangulation<T, TNearPointLocator>::trianglesAt(const V2d<T>& pos) const
{
    array<TriInd, 2> out = {noNeighbor, noNeighbor};
    for(TriInd i = TriInd(0); i < TriInd(triangles.size()); ++i)
    {
        const Triangle& t = triangles[i];
        const V2d<T>& v1 = vertices[t.vertices[0]];
        const V2d<T>& v2 = vertices[t.vertices[1]];
        const V2d<T>& v3 = vertices[t.vertices[2]];
        const PtTriLocation::Enum loc = locatePointTriangle(pos, v1, v2, v3);
        if(loc == PtTriLocation::Outside)
            continue;
        out[0] = i;
        if(isOnEdge(loc))
            out[1] = t.neighbors[edgeNeighbor(loc)];
        return out;
    }
    throw std::runtime_error("No triangle was found at position");
}

template <typename T, typename TNearPointLocator>
TriInd Triangulation<T, TNearPointLocator>::walkTriangles(
    const VertInd startVertex,
    const V2d<T>& pos) const
{
    // begin walk in search of triangle at pos
    TriInd currTri = vertTris[startVertex][0];
#ifdef CDT_USE_BOOST
    TriIndFlatUSet visited;
#else
    TriIndUSet visited;
#endif
    bool found = false;
    while(!found)
    {
        const Triangle& t = triangles[currTri];
        found = true;
        // stochastic offset to randomize which edge we check first
        const Index offset(detail::randGenerator() % 3);
        for(Index i_(0); i_ < Index(3); ++i_)
        {
            const Index i((i_ + offset) % 3);
            const V2d<T>& vStart = vertices[t.vertices[i]];
            const V2d<T>& vEnd = vertices[t.vertices[ccw(i)]];
            const PtLineLocation::Enum edgeCheck =
                locatePointLine(pos, vStart, vEnd);
            if(edgeCheck == PtLineLocation::Right &&
               t.neighbors[i] != noNeighbor &&
               visited.insert(t.neighbors[i]).second)
            {
                found = false;
                currTri = t.neighbors[i];
                break;
            }
        }
    }
    return currTri;
}

template <typename T, typename TNearPointLocator>
array<TriInd, 2> Triangulation<T, TNearPointLocator>::walkingSearchTrianglesAt(
    const V2d<T>& pos) const
{
    array<TriInd, 2> out = {noNeighbor, noNeighbor};
    // Query  for a vertex close to pos, to start the search
    const VertInd startVertex = m_nearPtLocator.nearPoint(pos, vertices);
    const TriInd iT = walkTriangles(startVertex, pos);
    // Finished walk, locate point in current triangle
    const Triangle& t = triangles[iT];
    const V2d<T>& v1 = vertices[t.vertices[0]];
    const V2d<T>& v2 = vertices[t.vertices[1]];
    const V2d<T>& v3 = vertices[t.vertices[2]];
    const PtTriLocation::Enum loc = locatePointTriangle(pos, v1, v2, v3);
    if(loc == PtTriLocation::Outside)
        throw std::runtime_error("No triangle was found at position");
    out[0] = iT;
    if(isOnEdge(loc))
        out[1] = t.neighbors[edgeNeighbor(loc)];
    return out;
}

/* Flip edge between T and Topo:
 *
 *                v4         | - old edge
 *               /|\         ~ - new edge
 *              / | \
 *          n3 /  T' \ n4
 *            /   |   \
 *           /    |    \
 *     T -> v1~~~~~~~~~v3 <- Topo
 *           \    |    /
 *            \   |   /
 *          n1 \Topo'/ n2
 *              \ | /
 *               \|/
 *                v2
 */
template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::flipEdge(
    const TriInd iT,
    const TriInd iTopo)
{
    Triangle& t = triangles[iT];
    Triangle& tOpo = triangles[iTopo];
    const array<TriInd, 3>& triNs = t.neighbors;
    const array<TriInd, 3>& triOpoNs = tOpo.neighbors;
    const array<VertInd, 3>& triVs = t.vertices;
    const array<VertInd, 3>& triOpoVs = tOpo.vertices;
    // find vertices and neighbors
    Index i = opposedVertexInd(t, iTopo);
    const VertInd v1 = triVs[i];
    const VertInd v2 = triVs[ccw(i)];
    const TriInd n1 = triNs[i];
    const TriInd n3 = triNs[cw(i)];
    i = opposedVertexInd(tOpo, iT);
    const VertInd v3 = triOpoVs[i];
    const VertInd v4 = triOpoVs[ccw(i)];
    const TriInd n4 = triOpoNs[i];
    const TriInd n2 = triOpoNs[cw(i)];
    // change vertices and neighbors
    using detail::arr3;
    t = Triangle::make(arr3(v4, v1, v3), arr3(n3, iTopo, n4));
    tOpo = Triangle::make(arr3(v2, v3, v1), arr3(n2, iT, n1));
    // adjust neighboring triangles and vertices
    changeNeighbor(n1, iT, iTopo);
    changeNeighbor(n4, iTopo, iT);
    addAdjacentTriangle(v1, iTopo);
    addAdjacentTriangle(v3, iT);
    removeAdjacentTriangle(v2, iT);
    removeAdjacentTriangle(v4, iTopo);
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::changeNeighbor(
    const TriInd iT,
    const TriInd oldNeighbor,
    const TriInd newNeighbor)
{
    if(iT == noNeighbor)
        return;
    Triangle& t = triangles[iT];
    t.neighbors[neighborInd(t, oldNeighbor)] = newNeighbor;
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::addAdjacentTriangle(
    const VertInd iVertex,
    const TriInd iTriangle)
{
    vertTris[iVertex].push_back(iTriangle);
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::addAdjacentTriangles(
    const VertInd iVertex,
    const TriInd iT1,
    const TriInd iT2,
    const TriInd iT3)
{
    TriIndVec& vTris = vertTris[iVertex];
    vTris.reserve(vTris.size() + 3);
    vTris.push_back(iT1);
    vTris.push_back(iT2);
    vTris.push_back(iT3);
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::addAdjacentTriangles(
    const VertInd iVertex,
    const TriInd iT1,
    const TriInd iT2,
    const TriInd iT3,
    const TriInd iT4)
{
    TriIndVec& vTris = vertTris[iVertex];
    vTris.reserve(vTris.size() + 4);
    vTris.push_back(iT1);
    vTris.push_back(iT2);
    vTris.push_back(iT3);
    vTris.push_back(iT4);
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::removeAdjacentTriangle(
    const VertInd iVertex,
    const TriInd iTriangle)
{
    std::vector<TriInd>& tris = vertTris[iVertex];
    tris.erase(std::find(tris.begin(), tris.end(), iTriangle));
}

/// Split points collection to points before and after given vertex index
inline std::pair<std::vector<VertInd>, std::vector<VertInd> >
splitPseudopolygon(const VertInd vi, const std::vector<VertInd>& points)
{
    std::pair<std::vector<VertInd>, std::vector<VertInd> > out;
    std::vector<VertInd>::const_iterator it;
    for(it = points.begin(); vi != *it; ++it)
        out.first.push_back(*it);
    for(it = it + 1; it != points.end(); ++it)
        out.second.push_back(*it);
    return out;
}

template <typename T, typename TNearPointLocator>
TriInd Triangulation<T, TNearPointLocator>::triangulatePseudopolygon(
    const VertInd ia,
    const VertInd ib,
    const std::vector<VertInd>& points)
{
    if(points.empty())
        return pseudopolyOuterTriangle(ia, ib);
    const VertInd ic = findDelaunayPoint(ia, ib, points);
    const std::pair<std::vector<VertInd>, std::vector<VertInd> > splitted =
        splitPseudopolygon(ic, points);
    // triangulate splitted pseudo-polygons
    TriInd iT2 = triangulatePseudopolygon(ic, ib, splitted.second);
    TriInd iT1 = triangulatePseudopolygon(ia, ic, splitted.first);
    // add new triangle
    const Triangle t = {{ia, ib, ic}, {noNeighbor, iT2, iT1}};
    const TriInd iT = addTriangle(t);
    // adjust neighboring triangles and vertices
    if(iT1 != noNeighbor)
    {
        if(splitted.first.empty())
            changeNeighbor(iT1, ia, ic, iT);
        else
            triangles[iT1].neighbors[0] = iT;
    }
    if(iT2 != noNeighbor)
    {
        if(splitted.second.empty())
            changeNeighbor(iT2, ic, ib, iT);
        else
            triangles[iT2].neighbors[0] = iT;
    }
    addAdjacentTriangle(ia, iT);
    addAdjacentTriangle(ib, iT);
    addAdjacentTriangle(ic, iT);

    return iT;
}

template <typename T, typename TNearPointLocator>
VertInd Triangulation<T, TNearPointLocator>::findDelaunayPoint(
    const VertInd ia,
    const VertInd ib,
    const std::vector<VertInd>& points) const
{
    assert(!points.empty());
    const V2d<T>& a = vertices[ia];
    const V2d<T>& b = vertices[ib];
    VertInd ic = points.front();
    V2d<T> c = vertices[ic];
    typedef std::vector<VertInd>::const_iterator CIt;
    for(CIt it = points.begin() + 1; it != points.end(); ++it)
    {
        const V2d<T> v = vertices[*it];
        if(!isInCircumcircle(v, a, b, c))
            continue;
        ic = *it;
        c = vertices[ic];
    }
    return ic;
}

template <typename T, typename TNearPointLocator>
TriInd Triangulation<T, TNearPointLocator>::pseudopolyOuterTriangle(
    const VertInd ia,
    const VertInd ib) const
{
    const std::vector<TriInd>& aTris = vertTris[ia];
    const std::vector<TriInd>& bTris = vertTris[ib];
    typedef std::vector<TriInd>::const_iterator TriIndCit;
    for(TriIndCit it = aTris.begin(); it != aTris.end(); ++it)
        if(std::find(bTris.begin(), bTris.end(), *it) != bTris.end())
            return *it;
    return noNeighbor;
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::insertVertices(
    const std::vector<V2d<T> >& newVertices)
{
    return insertVertices(
        newVertices.begin(), newVertices.end(), getX_V2d<T>, getY_V2d<T>);
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
    for(std::vector<Edge>::iterator it = edges.begin(); it != edges.end(); ++it)
    {
        *it = Edge(mapping[it->v1()], mapping[it->v2()]); // remap
    }
}

template <typename T>
DuplicatesInfo RemoveDuplicatesAndRemapEdges(
    std::vector<V2d<T> >& vertices,
    std::vector<Edge>& edges)
{
    return RemoveDuplicatesAndRemapEdges<T>(
        vertices, edges, getX_V2d<T>, getY_V2d<T>);
}

CDT_INLINE_IF_HEADER_ONLY
unordered_map<TriInd, LayerDepth> PeelLayer(
    std::stack<TriInd> seeds,
    const TriangleVec& triangles,
    const EdgeUSet& fixedEdges,
    const unordered_map<Edge, BoundaryOverlapCount>& overlapCount,
    const LayerDepth layerDepth,
    std::vector<LayerDepth>& triDepths)
{
    unordered_map<TriInd, LayerDepth> behindBoundary;
    while(!seeds.empty())
    {
        const TriInd iT = seeds.top();
        seeds.pop();
        triDepths[iT] = layerDepth;
        behindBoundary.erase(iT);
        const Triangle& t = triangles[iT];
        for(Index i(0); i < Index(3); ++i)
        {
            const Edge opEdge(t.vertices[ccw(i)], t.vertices[cw(i)]);
            const TriInd iN = t.neighbors[opoNbr(i)];
            if(iN == noNeighbor || triDepths[iN] <= layerDepth)
                continue;
            if(fixedEdges.count(opEdge))
            {
                const unordered_map<Edge, LayerDepth>::const_iterator cit =
                    overlapCount.find(opEdge);
                const LayerDepth triDepth = cit == overlapCount.end()
                                                ? layerDepth + 1
                                                : layerDepth + cit->second + 1;
                behindBoundary[iN] = triDepth;
                continue;
            }
            seeds.push(iN);
        }
    }
    return behindBoundary;
}

CDT_INLINE_IF_HEADER_ONLY
TriIndUSet PeelLayer(
    std::stack<TriInd> seeds,
    const TriangleVec& triangles,
    const EdgeUSet& fixedEdges,
    const LayerDepth layerDepth,
    std::vector<LayerDepth>& triDepths)
{
    TriIndUSet behindBoundary;
    while(!seeds.empty())
    {
        const TriInd iT = seeds.top();
        seeds.pop();
        triDepths[iT] = layerDepth;
        behindBoundary.erase(iT);
        const Triangle& t = triangles[iT];
        for(Index i(0); i < Index(3); ++i)
        {
            const Edge opEdge(t.vertices[ccw(i)], t.vertices[cw(i)]);
            const TriInd iN = t.neighbors[opoNbr(i)];
            if(iN == noNeighbor || triDepths[iN] <= layerDepth)
                continue;
            if(fixedEdges.count(opEdge))
            {
                behindBoundary.insert(iN);
                continue;
            }
            seeds.push(iN);
        }
    }
    return behindBoundary;
}

CDT_INLINE_IF_HEADER_ONLY
std::vector<LayerDepth> CalculateTriangleDepths(
    const TriInd seed,
    const TriangleVec& triangles,
    const EdgeUSet& fixedEdges,
    const unordered_map<Edge, BoundaryOverlapCount>& overlapCount)
{
    std::vector<LayerDepth> triDepths(
        triangles.size(), std::numeric_limits<LayerDepth>::max());
    std::stack<TriInd> seeds(TriDeque(1, seed));
    LayerDepth layerDepth = 0;
    LayerDepth deepestSeedDepth = 0;

    unordered_map<LayerDepth, TriIndUSet> seedsByDepth;
    do
    {
        const unordered_map<TriInd, LayerDepth>& newSeeds = PeelLayer(
            seeds, triangles, fixedEdges, overlapCount, layerDepth, triDepths);

        seedsByDepth.erase(layerDepth);
        typedef unordered_map<TriInd, LayerDepth>::const_iterator Iter;
        for(Iter it = newSeeds.begin(); it != newSeeds.end(); ++it)
        {
            deepestSeedDepth = std::max(deepestSeedDepth, it->second);
            seedsByDepth[it->second].insert(it->first);
        }
        const TriIndUSet& nextLayerSeeds = seedsByDepth[layerDepth + 1];
        seeds = std::stack<TriInd>(
            TriDeque(nextLayerSeeds.begin(), nextLayerSeeds.end()));
        ++layerDepth;
    } while(!seeds.empty() || deepestSeedDepth > layerDepth);

    return triDepths;
}

CDT_INLINE_IF_HEADER_ONLY
std::vector<LayerDepth> CalculateTriangleDepths(
    const TriInd seed,
    const TriangleVec& triangles,
    const EdgeUSet& fixedEdges)
{
    std::vector<LayerDepth> triDepths(
        triangles.size(), std::numeric_limits<LayerDepth>::max());
    std::stack<TriInd> seeds(TriDeque(1, seed));
    LayerDepth layerDepth = 0;

    do
    {
        const TriIndUSet& newSeeds =
            PeelLayer(seeds, triangles, fixedEdges, layerDepth++, triDepths);
        seeds = std::stack<TriInd>(TriDeque(newSeeds.begin(), newSeeds.end()));
    } while(!seeds.empty());

    return triDepths;
}

CDT_INLINE_IF_HEADER_ONLY EdgeUSet
extractEdgesFromTriangles(const TriangleVec& triangles)
{
    EdgeUSet edges;
    typedef TriangleVec::const_iterator CIt;
    for(CIt t = triangles.begin(); t != triangles.end(); ++t)
    {
        edges.insert(Edge(VertInd(t->vertices[0]), VertInd(t->vertices[1])));
        edges.insert(Edge(VertInd(t->vertices[1]), VertInd(t->vertices[2])));
        edges.insert(Edge(VertInd(t->vertices[2]), VertInd(t->vertices[0])));
    }
    return edges;
}

} // namespace CDT
