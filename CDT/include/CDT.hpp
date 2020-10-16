/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */
#include "CDT.h"

#include "remove_at.hpp"
#include <algorithm>

#ifdef CDT_CXX11_IS_SUPPORTED
namespace std
{

template <typename T>
struct hash<CDT::V2d<T> >
{
    size_t operator()(const CDT::V2d<T>& xy) const
    {
        return hash<T>()(xy.x) ^ hash<T>()(xy.y);
    }
};

} // namespace std

#endif

namespace CDT
{

namespace detail
{
static mt19937 randGen(9001);

// needed for c++03 compatibility (no uniform initialization available)
template <typename T>
array<T, 3> arr3(const T& v0, const T& v1, const T& v2)
{
    const array<T, 3> out = {v0, v1, v2};
    return out;
}

} // namespace detail

template <typename T>
Triangulation<T>::Triangulation(
    const FindingClosestPoint::Enum closestPtMode,
    const size_t nRandSamples)
    : m_nRandSamples(nRandSamples)
    , m_closestPtMode(closestPtMode)
{}

template <typename T>
void Triangulation<T>::changeNeighbor(
    const TriInd iT,
    const VertInd iVedge1,
    const VertInd iVedge2,
    const TriInd newNeighbor)
{
    Triangle& t = triangles[iT];
    t.neighbors[opposedTriangleInd(t, iVedge1, iVedge2)] = newNeighbor;
}

template <typename T>
void Triangulation<T>::eraseDummies()
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
    typedef typename VertexVec::iterator VertIt;
    for(VertIt v = vertices.begin(); v != vertices.end(); ++v)
    {
        TriIndVec& vTris = v->triangles;
        for(TriIndVec::iterator iT = vTris.begin(); iT != vTris.end(); ++iT)
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

template <typename T>
void Triangulation<T>::eraseSuperTriangleVertices()
{
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

    vertices = std::vector<Vertex<T> >(vertices.begin() + 3, vertices.end());
}

template <typename T>
void Triangulation<T>::eraseSuperTriangle()
{
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

template <typename T>
void Triangulation<T>::eraseOuterTriangles()
{
    // make dummy triangles adjacent to super-triangle's vertices
    const std::stack<TriInd> seed(
        std::deque<TriInd>(1, vertices[0].triangles.front()));
    const TriIndUSet toErase = growToBoundary(seed);
    eraseTrianglesAtIndices(toErase.begin(), toErase.end());
    eraseSuperTriangleVertices();
}

template <typename T>
void Triangulation<T>::eraseOuterTrianglesAndHoles()
{
    std::stack<TriInd> seeds;
    seeds.push(vertices[0].triangles.front());
    std::pair<TriIndUSet, TriIndUSet> growResult;
    TriIndVec toErase;
    TriIndUSet traversed;
    bool isOuterRegion = true;
    do
    {
        growResult = growToBoundaryExt(seeds, traversed);
        const TriIndUSet& region = growResult.first;
        const TriIndUSet& newSeeds = growResult.second;
        if(isOuterRegion)
            toErase.insert(toErase.end(), region.begin(), region.end());
        isOuterRegion = !isOuterRegion;
        seeds = std::stack<TriInd>(
            std::deque<TriInd>(newSeeds.begin(), newSeeds.end()));
    } while(!seeds.empty());
    eraseTrianglesAtIndices(toErase.begin(), toErase.end());
    eraseSuperTriangleVertices();
}

template <typename T>
template <typename TriIndexIter>
void Triangulation<T>::eraseTrianglesAtIndices(
    TriIndexIter first,
    TriIndexIter last)
{
    for(; first != last; ++first)
        makeDummy(*first);
    eraseDummies();
}

template <typename T>
TriIndUSet Triangulation<T>::growToBoundary(std::stack<TriInd> seeds) const
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

template <typename T>
std::pair<TriIndUSet, TriIndUSet> Triangulation<T>::growToBoundaryExt(
    std::stack<TriInd> seeds,
    TriIndUSet& traversed) const
{
    TriIndUSet newTraversed;
    TriIndUSet behindBoundary;
    while(!seeds.empty())
    {
        const TriInd iT = seeds.top();
        seeds.pop();
        traversed.insert(iT);
        newTraversed.insert(iT);
        behindBoundary.erase(iT);
        const Triangle& t = triangles[iT];
        for(Index i(0); i < Index(3); ++i)
        {
            const Edge opEdge(t.vertices[ccw(i)], t.vertices[cw(i)]);
            const TriInd iN = t.neighbors[opoNbr(i)];
            if(iN == noNeighbor || traversed.count(iN))
                continue;
            if(fixedEdges.count(opEdge))
            {
                behindBoundary.insert(iN);
                continue;
            }
            seeds.push(iN);
        }
    }
    return std::make_pair(newTraversed, behindBoundary);
}

template <typename T>
void Triangulation<T>::makeDummy(const TriInd iT)
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

template <typename T>
TriInd Triangulation<T>::addTriangle(const Triangle& t)
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

template <typename T>
TriInd Triangulation<T>::addTriangle()
{
    if(m_dummyTris.empty())
    {
        const Triangle dummy = {{noVertex, noVertex, noVertex},
                                {noNeighbor, noNeighbor, noNeighbor}};
        triangles.push_back(dummy);
        return TriInd(triangles.size() - 1);
    }
    const TriInd nxtDummy = m_dummyTris.back();
    m_dummyTris.pop_back();
    return nxtDummy;
}

template <typename T>
void Triangulation<T>::insertEdges(const std::vector<Edge>& edges)
{
    typedef std::vector<Edge>::const_iterator ECit;
    for(ECit e = edges.begin(); e != edges.end(); ++e)
    {
        // +3 to account for super-triangle vertices
        insertEdge(Edge(VertInd(e->v1() + 3), VertInd(e->v2() + 3)));
    }
    eraseDummies();
}

template <typename T>
void Triangulation<T>::insertEdge(Edge edge)
{
    const VertInd iA = edge.v1();
    VertInd iB = edge.v2();
    if(iA == iB) // edge connects a vertex to itself
        return;
    const Vertex<T>& a = vertices[iA];
    const Vertex<T>& b = vertices[iB];
    if(verticesShareEdge(a, b))
    {
        fixedEdges.insert(Edge(iA, iB));
        return;
    }
    TriInd iT;
    VertInd iVleft, iVright;
    tie(iT, iVleft, iVright) =
        intersectedTriangle(iA, a.triangles, a.pos, b.pos);
    // if one of the triangle vertices is on the edge, move edge start
    if(iT == noNeighbor)
    {
        fixedEdges.insert(Edge(iA, iVleft));
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
        const Vertex<T> vOpo = vertices[iVopo];

        intersected.push_back(iTopo);
        iT = iTopo;
        t = triangles[iT];

        PtLineLocation::Enum loc = locatePointLine(vOpo.pos, a.pos, b.pos);
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
    fixedEdges.insert(Edge(iA, iB));
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
template <typename T>
tuple<TriInd, VertInd, VertInd> Triangulation<T>::intersectedTriangle(
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
        const PtLineLocation::Enum locP1 =
            locatePointLine(vertices[iP1].pos, a, b);
        const PtLineLocation::Enum locP2 =
            locatePointLine(vertices[iP2].pos, a, b);
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

template <typename T>
void Triangulation<T>::addSuperTriangle(const Box2d<T>& box)
{
    const V2d<T> center = {(box.min.x + box.max.x) / T(2),
                           (box.min.y + box.max.y) / T(2)};
    const T w = box.max.x - box.min.x;
    const T h = box.max.y - box.min.y;
    T r = std::sqrt(w * w + h * h) / 2.0; // incircle radius
    r *= 1.1;
    const T R = 2.0 * r;                        // excircle radius
    const T shiftX = R * std::sqrt(T(3)) / 2.0; // R * cos(30 deg)
    const V2d<T> posV1 = {center.x - shiftX, center.y - r};
    const V2d<T> posV2 = {center.x + shiftX, center.y - r};
    const V2d<T> posV3 = {center.x, center.y + R};
    vertices.push_back(Vertex<T>::make(posV1, TriInd(0)));
    vertices.push_back(Vertex<T>::make(posV2, TriInd(0)));
    vertices.push_back(Vertex<T>::make(posV3, TriInd(0)));
    const Triangle superTri = {{VertInd(0), VertInd(1), VertInd(2)},
                               {noNeighbor, noNeighbor, noNeighbor}};
    addTriangle(superTri);
#ifndef CDT_DONT_USE_BOOST_RTREE
    if(m_closestPtMode == FindingClosestPoint::BoostRTree)
    {
        m_rtree.addPoint(posV1, VertInd(0));
        m_rtree.addPoint(posV2, VertInd(1));
        m_rtree.addPoint(posV3, VertInd(2));
    }
#endif
}

template <typename T>
void Triangulation<T>::insertVertex(const V2d<T>& pos)
{
    const VertInd iVert(vertices.size());
    array<TriInd, 2> trisAt = walkingSearchTrianglesAt(pos);
    std::stack<TriInd> triStack =
        trisAt[1] == noNeighbor ? insertPointInTriangle(pos, trisAt[0])
                                : insertPointOnEdge(pos, trisAt[0], trisAt[1]);
    while(!triStack.empty())
    {
        const TriInd iT = triStack.top();
        triStack.pop();

        const Triangle& t = triangles[iT];
        const TriInd iTopo = opposedTriangle(t, iVert);
        if(iTopo == noNeighbor)
            continue;
        if(isFlipNeeded(pos, iT, iTopo, iVert))
        {
            flipEdge(iT, iTopo);
            triStack.push(iT);
            triStack.push(iTopo);
        }
    }
#ifndef CDT_DONT_USE_BOOST_RTREE
    if(m_closestPtMode == FindingClosestPoint::BoostRTree)
        m_rtree.addPoint(pos, iVert);
#endif
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
template <typename T>
bool Triangulation<T>::isFlipNeeded(
    const V2d<T>& pos,
    const TriInd iT,
    const TriInd iTopo,
    const VertInd iVert) const
{
    const Triangle& tOpo = triangles[iTopo];
    const Index i = opposedVertexInd(tOpo, iT);
    const VertInd iVopo = tOpo.vertices[i];
    if(iVert < 3 && iVopo < 3) // opposed vertices belong to super-triangle
        return false;          // no flip is needed
    const VertInd iVcw = tOpo.vertices[cw(i)];
    const VertInd iVccw = tOpo.vertices[ccw(i)];
    const V2d<T>& v1 = vertices[iVcw].pos;
    const V2d<T>& v2 = vertices[iVopo].pos;
    const V2d<T>& v3 = vertices[iVccw].pos;
    if(iVcw < 3)
        return locatePointLine(v1, v2, v3) == locatePointLine(pos, v2, v3);
    if(iVccw < 3)
        return locatePointLine(v3, v1, v2) == locatePointLine(pos, v1, v2);
    return isInCircumcircle(pos, v1, v2, v3);
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
template <typename T>
std::stack<TriInd>
Triangulation<T>::insertPointInTriangle(const V2d<T>& pos, const TriInd iT)
{
    const VertInd v(vertices.size());
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
    vertices.push_back(Vertex<T>::makeInTriangle(pos, iT, iNewT1, iNewT2));
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
template <typename T>
std::stack<TriInd> Triangulation<T>::insertPointOnEdge(
    const V2d<T>& pos,
    const TriInd iT1,
    const TriInd iT2)
{
    const VertInd v(vertices.size());
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
    vertices.push_back(Vertex<T>::makeOnEdge(pos, iT1, iTnew2, iT2, iTnew1));
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

template <typename T>
array<TriInd, 2> Triangulation<T>::trianglesAt(const V2d<T>& pos) const
{
    array<TriInd, 2> out = {noNeighbor, noNeighbor};
    for(TriInd i = TriInd(0); i < TriInd(triangles.size()); ++i)
    {
        const Triangle& t = triangles[i];
        const V2d<T> v1 = vertices[t.vertices[0]].pos;
        const V2d<T> v2 = vertices[t.vertices[1]].pos;
        const V2d<T> v3 = vertices[t.vertices[2]].pos;
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

template <typename T>
TriInd Triangulation<T>::walkTriangles(
    const VertInd startVertex,
    const V2d<T>& pos) const
{
    // begin walk in search of triangle at pos
    TriInd currTri = vertices[startVertex].triangles[0];
    TriIndUSet visited;
    bool found = false;
    while(!found)
    {
        const Triangle& t = triangles[currTri];
        found = true;
        // stochastic offset to randomize which edge we check first
        const Index offset(detail::randGen() % 3);
        for(Index i_(0); i_ < Index(3); ++i_)
        {
            const Index i((i_ + offset) % 3);
            const V2d<T> vStart = vertices[t.vertices[i]].pos;
            const V2d<T> vEnd = vertices[t.vertices[ccw(i)]].pos;
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

template <typename T>
array<TriInd, 2>
Triangulation<T>::walkingSearchTrianglesAt(const V2d<T>& pos) const
{
    array<TriInd, 2> out = {noNeighbor, noNeighbor};
    // Query RTree for a vertex close to pos, to start the search
#ifndef CDT_DONT_USE_BOOST_RTREE
    const VertInd startVertex =
        m_closestPtMode == FindingClosestPoint::BoostRTree
            ? nearestVertexRtree(pos)
            : nearestVertexRand(pos, m_nRandSamples);
#else
    const VertInd startVertex = nearestVertexRand(pos, m_nRandSamples);
#endif
    const TriInd iT = walkTriangles(startVertex, pos);
    // Finished walk, locate point in current triangle
    const Triangle& t = triangles[iT];
    const V2d<T> v1 = vertices[t.vertices[0]].pos;
    const V2d<T> v2 = vertices[t.vertices[1]].pos;
    const V2d<T> v3 = vertices[t.vertices[2]].pos;
    const PtTriLocation::Enum loc = locatePointTriangle(pos, v1, v2, v3);
    if(loc == PtTriLocation::Outside)
        throw std::runtime_error("No triangle was found at position");
    out[0] = iT;
    if(isOnEdge(loc))
        out[1] = t.neighbors[edgeNeighbor(loc)];
    return out;
}

#ifndef CDT_DONT_USE_BOOST_RTREE
template <typename T>
VertInd Triangulation<T>::nearestVertexRtree(const V2d<T>& pos) const
{
    return m_rtree.nearestPoint(pos);
}
#endif

template <typename T>
VertInd Triangulation<T>::nearestVertexRand(
    const V2d<T>& pos,
    const std::size_t nSamples) const
{
    // start search at a vertex close to pos based on random sampling
    VertInd out(detail::randGen() % vertices.size());
    T minDist = distance(vertices[out].pos, pos);
    for(std::size_t iSample = 0; iSample < nSamples; ++iSample)
    {
        const VertInd candidate(detail::randGen() % vertices.size());
        const T candidateDist = distance(vertices[candidate].pos, pos);
        if(candidateDist < minDist)
        {
            minDist = candidateDist;
            out = candidate;
        }
    }
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
template <typename T>
void Triangulation<T>::flipEdge(const TriInd iT, const TriInd iTopo)
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

template <typename T>
void Triangulation<T>::changeNeighbor(
    const TriInd iT,
    const TriInd oldNeighbor,
    const TriInd newNeighbor)
{
    if(iT == noNeighbor)
        return;
    Triangle& t = triangles[iT];
    t.neighbors[neighborInd(t, oldNeighbor)] = newNeighbor;
}

template <typename T>
void Triangulation<T>::addAdjacentTriangle(
    const VertInd iVertex,
    const TriInd iTriangle)
{
    vertices[iVertex].triangles.push_back(iTriangle);
}

template <typename T>
void Triangulation<T>::removeAdjacentTriangle(
    const VertInd iVertex,
    const TriInd iTriangle)
{
    std::vector<TriInd>& tris = vertices[iVertex].triangles;
    tris.erase(std::find(tris.begin(), tris.end(), iTriangle));
}

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

template <typename T>
TriInd Triangulation<T>::triangulatePseudopolygon(
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

template <typename T>
VertInd Triangulation<T>::findDelaunayPoint(
    const VertInd ia,
    const VertInd ib,
    const std::vector<VertInd>& points) const
{
    assert(!points.empty());
    const V2d<T>& a = vertices[ia].pos;
    const V2d<T>& b = vertices[ib].pos;
    VertInd ic = points.front();
    V2d<T> c = vertices[ic].pos;
    typedef std::vector<VertInd>::const_iterator CIt;
    for(CIt it = points.begin() + 1; it != points.end(); ++it)
    {
        const V2d<T> v = vertices[*it].pos;
        if(!isInCircumcircle(v, a, b, c))
            continue;
        ic = *it;
        c = vertices[ic].pos;
    }
    return ic;
}

template <typename T>
TriInd Triangulation<T>::pseudopolyOuterTriangle(
    const VertInd ia,
    const VertInd ib) const
{
    const std::vector<TriInd>& aTris = vertices[ia].triangles;
    const std::vector<TriInd>& bTris = vertices[ib].triangles;
    typedef std::vector<TriInd>::const_iterator TriIndCit;
    for(TriIndCit it = aTris.begin(); it != aTris.end(); ++it)
        if(std::find(bTris.begin(), bTris.end(), *it) != bTris.end())
            return *it;
    return noNeighbor;
}

template <typename T>
void Triangulation<T>::insertVertices(const std::vector<V2d<T> >& newVertices)
{
    if(vertices.empty())
        addSuperTriangle(Box2d<T>::envelop(newVertices));
    vertices.reserve(vertices.size() + newVertices.size());
    typedef typename std::vector<V2d<T> >::const_iterator Cit;
    for(Cit it = newVertices.begin(); it != newVertices.end(); ++it)
        insertVertex(*it);
}

template <typename T>
std::vector<std::size_t> RemoveDuplicates(std::vector<V2d<T> >& vertices)
{
    typedef unordered_map<V2d<T>, std::size_t> PosToIndex;
    PosToIndex uniqueVerts;
    std::vector<std::size_t> mapping(vertices.size());
    std::vector<std::size_t> removedDuplicateIndices;

    for(std::size_t iIn = 0, iOut = iIn; iIn < vertices.size(); ++iIn)
    {
        typename PosToIndex::const_iterator it;
        bool isUnique;
        tie(it, isUnique) =
            uniqueVerts.insert(std::make_pair(vertices[iIn], iOut));
        if(isUnique)
        {
            mapping[iIn] = iOut++;
            continue;
        }
        mapping[iIn] = it->second; // found a duplicate
        removedDuplicateIndices.push_back(iIn);
    }

    // remove detected duplicates
    vertices.erase(
        remove_at(
            vertices.begin(),
            vertices.end(),
            removedDuplicateIndices.begin(),
            removedDuplicateIndices.end()),
        vertices.end());

    return mapping;
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
std::vector<std::size_t> RemoveDuplicatesAndRemapEdges(
    std::vector<V2d<T> >& vertices,
    std::vector<Edge>& edges)
{
    const std::vector<std::size_t> mapping = RemoveDuplicates(vertices);
    RemapEdges(edges, mapping);
    return mapping;
}

} // namespace CDT
