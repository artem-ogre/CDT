/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/**
 * @file
 * Triangulation class - implementation
 */

#include "Triangulation.h"

#include <algorithm>
#include <cassert>
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

namespace defaults
{

const std::size_t nTargetVerts = 0;
const SuperGeometryType::Enum superGeomType = SuperGeometryType::SuperTriangle;
const VertexInsertionOrder::Enum vertexInsertionOrder =
    VertexInsertionOrder::Randomized;
const IntersectingConstraintEdges::Enum intersectingEdgesStrategy =
    IntersectingConstraintEdges::Ignore;
const float minDistToConstraintEdge(0);

} // namespace defaults

} // namespace detail

template <typename T, typename TNearPointLocator>
Triangulation<T, TNearPointLocator>::Triangulation()
    : m_nTargetVerts(detail::defaults::nTargetVerts)
    , m_superGeomType(detail::defaults::superGeomType)
    , m_vertexInsertionOrder(detail::defaults::vertexInsertionOrder)
    , m_intersectingEdgesStrategy(detail::defaults::intersectingEdgesStrategy)
    , m_minDistToConstraintEdge(detail::defaults::minDistToConstraintEdge)
{}

template <typename T, typename TNearPointLocator>
Triangulation<T, TNearPointLocator>::Triangulation(
    const VertexInsertionOrder::Enum vertexInsertionOrder)
    : m_nTargetVerts(detail::defaults::nTargetVerts)
    , m_superGeomType(detail::defaults::superGeomType)
    , m_vertexInsertionOrder(vertexInsertionOrder)
    , m_intersectingEdgesStrategy(detail::defaults::intersectingEdgesStrategy)
    , m_minDistToConstraintEdge(detail::defaults::minDistToConstraintEdge)
{}

template <typename T, typename TNearPointLocator>
Triangulation<T, TNearPointLocator>::Triangulation(
    const VertexInsertionOrder::Enum vertexInsertionOrder,
    const IntersectingConstraintEdges::Enum intersectingEdgesStrategy,
    const T minDistToConstraintEdge)
    : m_nTargetVerts(detail::defaults::nTargetVerts)
    , m_superGeomType(detail::defaults::superGeomType)
    , m_vertexInsertionOrder(vertexInsertionOrder)
    , m_intersectingEdgesStrategy(intersectingEdgesStrategy)
    , m_minDistToConstraintEdge(minDistToConstraintEdge)
{}

template <typename T, typename TNearPointLocator>
Triangulation<T, TNearPointLocator>::Triangulation(
    const VertexInsertionOrder::Enum vertexInsertionOrder,
    const TNearPointLocator& nearPtLocator,
    const IntersectingConstraintEdges::Enum intersectingEdgesStrategy,
    const T minDistToConstraintEdge)
    : m_nTargetVerts(detail::defaults::nTargetVerts)
    , m_nearPtLocator(nearPtLocator)
    , m_superGeomType(detail::defaults::superGeomType)
    , m_vertexInsertionOrder(vertexInsertionOrder)
    , m_intersectingEdgesStrategy(intersectingEdgesStrategy)
    , m_minDistToConstraintEdge(minDistToConstraintEdge)
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
void Triangulation<T, TNearPointLocator>::eraseSuperTriangle()
{
    if(m_superGeomType != SuperGeometryType::SuperTriangle)
        return;
    // find triangles adjacent to super-triangle's vertices
    TriIndUSet toErase;
    toErase.reserve(
        vertTris[0].size() + vertTris[1].size() + vertTris[2].size());
    for(TriInd iT(0); iT < TriInd(triangles.size()); ++iT)
    {
        Triangle& t = triangles[iT];
        if(t.vertices[0] < 3 || t.vertices[1] < 3 || t.vertices[2] < 3)
            toErase.insert(iT);
    }
    finalizeTriangulation(toErase);
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::eraseOuterTriangles()
{
    // make dummy triangles adjacent to super-triangle's vertices
    const std::stack<TriInd> seed(std::deque<TriInd>(1, vertTris[0].front()));
    const TriIndUSet toErase = growToBoundary(seed);
    finalizeTriangulation(toErase);
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::eraseOuterTrianglesAndHoles()
{
    const std::vector<LayerDepth> triDepths = calculateTriangleDepths();
    TriIndUSet toErase;
    toErase.reserve(triangles.size());
    for(std::size_t iT = 0; iT != triangles.size(); ++iT)
    {
        if(triDepths[iT] % 2 == 0)
            toErase.insert(static_cast<TriInd>(iT));
    }
    finalizeTriangulation(toErase);
}

/// Remap removing super-triangle: subtract 3 from vertices
inline Edge RemapNoSuperTriangle(const Edge& e)
{
    return Edge(e.v1() - 3, e.v2() - 3);
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::removeTriangles(
    const TriIndUSet& removedTriangles)
{
    if(removedTriangles.empty())
        return;
    // remove triangles and calculate triangle index mapping
    TriIndUMap triIndMap;
    for(TriInd iT(0), iTnew(0); iT < TriInd(triangles.size()); ++iT)
    {
        if(removedTriangles.count(iT))
            continue;
        triIndMap[iT] = iTnew;
        triangles[iTnew] = triangles[iT];
        iTnew++;
    }
    triangles.erase(triangles.end() - removedTriangles.size(), triangles.end());
    // adjust triangles' neighbors
    vertTris = VerticesTriangles();
    for(TriInd iT = 0; iT < triangles.size(); ++iT)
    {
        Triangle& t = triangles[iT];
        // update neighbors to account for removed triangles
        NeighborsArr3& nn = t.neighbors;
        for(NeighborsArr3::iterator n = nn.begin(); n != nn.end(); ++n)
        {
            if(removedTriangles.count(*n))
            {
                *n = noNeighbor;
            }
            else if(*n != noNeighbor)
            {
                *n = triIndMap[*n];
            }
        }
    }
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::finalizeTriangulation(
    const TriIndUSet& removedTriangles)
{
    eraseDummies();
    // remove super-triangle
    if(m_superGeomType == SuperGeometryType::SuperTriangle)
    {
        vertices.erase(vertices.begin(), vertices.begin() + 3);
        if(removedTriangles.empty())
            vertTris.erase(vertTris.begin(), vertTris.begin() + 3);
        // Edge re-mapping
        { // fixed edges
            EdgeUSet updatedFixedEdges;
            typedef CDT::EdgeUSet::const_iterator It;
            for(It e = fixedEdges.begin(); e != fixedEdges.end(); ++e)
            {
                updatedFixedEdges.insert(RemapNoSuperTriangle(*e));
            }
            fixedEdges = updatedFixedEdges;
        }
        { // overlap count
            unordered_map<Edge, BoundaryOverlapCount> updatedOverlapCount;
            typedef unordered_map<Edge, BoundaryOverlapCount>::const_iterator
                It;
            for(It it = overlapCount.begin(); it != overlapCount.end(); ++it)
            {
                updatedOverlapCount.insert(std::make_pair(
                    RemapNoSuperTriangle(it->first), it->second));
            }
            overlapCount = updatedOverlapCount;
        }
        { // split edges mapping
            unordered_map<Edge, EdgeVec> updatedPieceToOriginals;
            typedef unordered_map<Edge, EdgeVec>::const_iterator It;
            for(It it = pieceToOriginals.begin(); it != pieceToOriginals.end();
                ++it)
            {
                EdgeVec ee = it->second;
                for(EdgeVec::iterator eeIt = ee.begin(); eeIt != ee.end();
                    ++eeIt)
                {
                    *eeIt = RemapNoSuperTriangle(*eeIt);
                }
                updatedPieceToOriginals.insert(
                    std::make_pair(RemapNoSuperTriangle(it->first), ee));
            }
            pieceToOriginals = updatedPieceToOriginals;
        }
    }
    // remove other triangles
    removeTriangles(removedTriangles);
    // adjust triangle vertices: account for removed super-triangle
    if(m_superGeomType == SuperGeometryType::SuperTriangle)
    {
        for(TriangleVec::iterator t = triangles.begin(); t != triangles.end();
            ++t)
        {
            VerticesArr3& vv = t->vertices;
            for(VerticesArr3::iterator v = vv.begin(); v != vv.end(); ++v)
            {
                *v -= 3;
            }
        }
    }
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::initializedWithCustomSuperGeometry()
{
    m_nearPtLocator.initialize(vertices);
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
void Triangulation<T, TNearPointLocator>::conformToEdges(
    const std::vector<Edge>& edges)
{
    conformToEdges(edges.begin(), edges.end(), edge_get_v1, edge_get_v2);
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::fixEdge(const Edge& edge)
{
    if(!fixedEdges.insert(edge).second)
    {
        ++overlapCount[edge]; // if edge is already fixed increment the counter
    }
}

namespace detail
{

// add element to 'to' if not already in 'to'
template <typename T, typename Allocator1>
void insert_unique(std::vector<T, Allocator1>& to, const T& elem)
{
    if(std::find(to.begin(), to.end(), elem) == to.end())
    {
        to.push_back(elem);
    }
}

// add elements of 'from' that are not present in 'to' to 'to'
template <typename T, typename Allocator1, typename Allocator2>
void insert_unique(
    std::vector<T, Allocator1>& to,
    const std::vector<T, Allocator2>& from)
{
    typedef typename std::vector<T, Allocator2>::const_iterator Cit;
    to.reserve(to.size() + from.size());
    for(Cit cit = from.begin(); cit != from.end(); ++cit)
    {
        insert_unique(to, *cit);
    }
}

} // namespace detail

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::fixEdge(
    const Edge& edge,
    const Edge& originalEdge)
{
    fixEdge(edge);
    if(edge != originalEdge)
        detail::insert_unique(pieceToOriginals[edge], originalEdge);
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::fixEdge(
    const Edge& edge,
    const BoundaryOverlapCount overlaps)
{
    fixedEdges.insert(edge);
    overlapCount[edge] = overlaps; // override overlap counter
}

namespace detail
{

template <typename T>
T lerp(const T& a, const T& b, const T t)
{
    return (T(1) - t) * a + t * b;
}

// Precondition: ab and cd intersect normally
template <typename T>
V2d<T> intersectionPosition(
    const V2d<T>& a,
    const V2d<T>& b,
    const V2d<T>& c,
    const V2d<T>& d)
{
    using namespace predicates::adaptive;
    // interpolate point on the shorter segment
    if(distanceSquared(a, b) < distanceSquared(c, d))
    {
        const T a_cd = orient2d(c.x, c.y, d.x, d.y, a.x, a.y);
        const T b_cd = orient2d(c.x, c.y, d.x, d.y, b.x, b.y);
        const T t = a_cd / (a_cd - b_cd);
        return V2d<T>::make(lerp(a.x, b.x, t), lerp(a.y, b.y, t));
    }
    else
    {
        const T c_ab = orient2d(a.x, a.y, b.x, b.y, c.x, c.y);
        const T d_ab = orient2d(a.x, a.y, b.x, b.y, d.x, d.y);
        const T t = c_ab / (c_ab - d_ab);
        return V2d<T>::make(lerp(c.x, d.x, t), lerp(c.y, d.y, t));
    }
}

} // namespace detail

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::insertEdge(
    const Edge edge,
    const Edge originalEdge)
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
        fixEdge(edge, originalEdge);
        return;
    }

    const T distanceTolerance =
        m_minDistToConstraintEdge == T(0)
            ? T(0)
            : m_minDistToConstraintEdge * distance(a, b);

    TriInd iT;
    VertInd iVleft, iVright;
    tie(iT, iVleft, iVright) =
        intersectedTriangle(iA, aTris, a, b, distanceTolerance);
    // if one of the triangle vertices is on the edge, move edge start
    if(iT == noNeighbor)
    {
        const Edge edgePart(iA, iVleft);
        fixEdge(edgePart, originalEdge);
        return insertEdge(Edge(iVleft, iB), originalEdge);
    }
    std::vector<TriInd> intersected(1, iT);
    std::vector<VertInd> ptsLeft(1, iVleft);
    std::vector<VertInd> ptsRight(1, iVright);
    VertInd iV = iA;
    Triangle t = triangles[iT];
    while(std::find(t.vertices.begin(), t.vertices.end(), iB) ==
          t.vertices.end())
    {
        const TriInd iTopo = opposedTriangle(t, iV);
        const Triangle& tOpo = triangles[iTopo];
        const VertInd iVopo = opposedVertex(tOpo, iT);
        const V2d<T> vOpo = vertices[iVopo];

        // Resolve intersection between two constraint edges if needed
        if(m_intersectingEdgesStrategy ==
               IntersectingConstraintEdges::Resolve &&
           fixedEdges.count(Edge(iVleft, iVright)))
        {
            const VertInd iNewVert = static_cast<VertInd>(vertices.size());

            // split constraint edge that already exists in triangulation
            const Edge splitEdge(iVleft, iVright);
            const Edge half1(iVleft, iNewVert);
            const Edge half2(iNewVert, iVright);
            const BoundaryOverlapCount overlaps = overlapCount[splitEdge];
            // remove the edge that will be split
            fixedEdges.erase(splitEdge);
            overlapCount.erase(splitEdge);
            // add split edge's halves
            fixEdge(half1, overlaps);
            fixEdge(half2, overlaps);
            // maintain piece-to-original mapping
            EdgeVec newOriginals(1, splitEdge);
            const unordered_map<Edge, EdgeVec>::const_iterator originalsIt =
                pieceToOriginals.find(splitEdge);
            if(originalsIt != pieceToOriginals.end())
            { // edge being split was split before: pass-through originals
                newOriginals = originalsIt->second;
                pieceToOriginals.erase(originalsIt);
            }
            detail::insert_unique(pieceToOriginals[half1], newOriginals);
            detail::insert_unique(pieceToOriginals[half2], newOriginals);

            // add a new point at the intersection of two constraint edges
            const V2d<T> newV = detail::intersectionPosition(
                vertices[iA],
                vertices[iB],
                vertices[iVleft],
                vertices[iVright]);
            addNewVertex(newV, TriIndVec());
            std::stack<TriInd> triStack =
                insertPointOnEdge(iNewVert, iT, iTopo);
            ensureDelaunayByEdgeFlips(newV, iNewVert, triStack);
            // TODO: is it's possible to re-use pseudo-polygons
            //  for inserting [iA, iNewVert] edge half?
            insertEdge(Edge(iA, iNewVert), originalEdge);
            insertEdge(Edge(iNewVert, iB), originalEdge);
            return;
        }

        intersected.push_back(iTopo);
        iT = iTopo;
        t = triangles[iT];

        const PtLineLocation::Enum loc =
            locatePointLine(vOpo, a, b, distanceTolerance);
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
    const TriInd iTleft =
        triangulatePseudopolygon(iA, iB, ptsLeft.begin(), ptsLeft.end());
    std::reverse(ptsRight.begin(), ptsRight.end());
    const TriInd iTright =
        triangulatePseudopolygon(iB, iA, ptsRight.begin(), ptsRight.end());
    changeNeighbor(iTleft, noNeighbor, iTright);
    changeNeighbor(iTright, noNeighbor, iTleft);

    if(iB != edge.v2()) // encountered point on the edge
    {
        // fix edge part
        const Edge edgePart(iA, iB);
        fixEdge(edgePart, originalEdge);
        return insertEdge(Edge(iB, edge.v2()), originalEdge);
    }
    else
    {
        fixEdge(edge, originalEdge);
    }
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::conformToEdge(
    const Edge edge,
    EdgeVec originalEdges,
    const BoundaryOverlapCount overlaps)
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
        overlaps > 0 ? fixEdge(edge, overlaps) : fixEdge(edge);
        // avoid marking edge as a part of itself
        if(!originalEdges.empty() && edge != originalEdges.front())
        {
            detail::insert_unique(pieceToOriginals[edge], originalEdges);
        }
        return;
    }

    const T distanceTolerance =
        m_minDistToConstraintEdge == T(0)
            ? T(0)
            : m_minDistToConstraintEdge * distance(a, b);
    TriInd iT;
    VertInd iVleft, iVright;
    tie(iT, iVleft, iVright) =
        intersectedTriangle(iA, aTris, a, b, distanceTolerance);
    // if one of the triangle vertices is on the edge, move edge start
    if(iT == noNeighbor)
    {
        const Edge edgePart(iA, iVleft);
        overlaps > 0 ? fixEdge(edgePart, overlaps) : fixEdge(edgePart);
        detail::insert_unique(pieceToOriginals[edgePart], originalEdges);
        return conformToEdge(Edge(iVleft, iB), originalEdges, overlaps);
    }

    VertInd iV = iA;
    Triangle t = triangles[iT];
    while(std::find(t.vertices.begin(), t.vertices.end(), iB) ==
          t.vertices.end())
    {
        const TriInd iTopo = opposedTriangle(t, iV);
        const Triangle& tOpo = triangles[iTopo];
        const VertInd iVopo = opposedVertex(tOpo, iT);
        const V2d<T> vOpo = vertices[iVopo];

        // Resolve intersection between two constraint edges if needed
        if(m_intersectingEdgesStrategy ==
               IntersectingConstraintEdges::Resolve &&
           fixedEdges.count(Edge(iVleft, iVright)))
        {
            const VertInd iNewVert = static_cast<VertInd>(vertices.size());

            // split constraint edge that already exists in triangulation
            const Edge splitEdge(iVleft, iVright);
            const Edge half1(iVleft, iNewVert);
            const Edge half2(iNewVert, iVright);
            const BoundaryOverlapCount overlaps = overlapCount[splitEdge];
            // remove the edge that will be split
            fixedEdges.erase(splitEdge);
            overlapCount.erase(splitEdge);
            // add split edge's halves
            fixEdge(half1, overlaps);
            fixEdge(half2, overlaps);
            // maintain piece-to-original mapping
            EdgeVec newOriginals(1, splitEdge);
            const unordered_map<Edge, EdgeVec>::const_iterator originalsIt =
                pieceToOriginals.find(splitEdge);
            if(originalsIt != pieceToOriginals.end())
            { // edge being split was split before: pass-through originals
                newOriginals = originalsIt->second;
                pieceToOriginals.erase(originalsIt);
            }
            detail::insert_unique(pieceToOriginals[half1], newOriginals);
            detail::insert_unique(pieceToOriginals[half2], newOriginals);

            // add a new point at the intersection of two constraint edges
            const V2d<T> newV = detail::intersectionPosition(
                vertices[iA],
                vertices[iB],
                vertices[iVleft],
                vertices[iVright]);
            addNewVertex(newV, TriIndVec());
            std::stack<TriInd> triStack =
                insertPointOnEdge(iNewVert, iT, iTopo);
            ensureDelaunayByEdgeFlips(newV, iNewVert, triStack);
            conformToEdge(Edge(iA, iNewVert), originalEdges, overlaps);
            conformToEdge(Edge(iNewVert, iB), originalEdges, overlaps);
            return;
        }

        iT = iTopo;
        t = triangles[iT];

        const PtLineLocation::Enum loc =
            locatePointLine(vOpo, a, b, distanceTolerance);
        if(loc == PtLineLocation::Left)
        {
            iV = iVleft;
            iVleft = iVopo;
        }
        else if(loc == PtLineLocation::Right)
        {
            iV = iVright;
            iVright = iVopo;
        }
        else // encountered point on the edge
            iB = iVopo;
    }
    /**/

    // add mid-point to triangulation
    const VertInd iMid = static_cast<VertInd>(vertices.size());
    const V2d<T>& start = vertices[iA];
    const V2d<T>& end = vertices[iB];
    addNewVertex(
        V2d<T>::make((start.x + end.x) / T(2), (start.y + end.y) / T(2)),
        TriIndVec());
    const std::vector<Edge> flippedFixedEdges =
        insertVertex_FlipFixedEdges(iMid);

    conformToEdge(Edge(iA, iMid), originalEdges, overlaps);
    conformToEdge(Edge(iMid, iB), originalEdges, overlaps);
    // re-introduce fixed edges that were flipped
    // and make sure overlap count is preserved
    for(std::vector<Edge>::const_iterator it = flippedFixedEdges.begin();
        it != flippedFixedEdges.end();
        ++it)
    {
        fixedEdges.erase(*it);

        BoundaryOverlapCount prevOverlaps = 0;
        const unordered_map<Edge, BoundaryOverlapCount>::const_iterator
            overlapsIt = overlapCount.find(*it);
        if(overlapsIt != overlapCount.end())
        {
            prevOverlaps = overlapsIt->second;
            overlapCount.erase(overlapsIt);
        }
        // override overlapping boundaries count when re-inserting an edge
        EdgeVec prevOriginals(1, *it);
        const unordered_map<Edge, EdgeVec>::const_iterator originalsIt =
            pieceToOriginals.find(*it);
        if(originalsIt != pieceToOriginals.end())
        {
            prevOriginals = originalsIt->second;
        }
        conformToEdge(*it, prevOriginals, prevOverlaps);
    }
    if(iB != edge.v2())
        conformToEdge(Edge(iB, edge.v2()), originalEdges, overlaps);
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
    const V2d<T>& b,
    const T orientationTolerance) const
{
    typedef std::vector<TriInd>::const_iterator TriIndCit;
    for(TriIndCit it = candidates.begin(); it != candidates.end(); ++it)
    {
        const TriInd iT = *it;
        const Triangle t = triangles[iT];
        const Index i = vertexInd(t, iA);
        const VertInd iP2 = t.vertices[ccw(i)];
        const T orientP2 = orient2D(vertices[iP2], a, b);
        const PtLineLocation::Enum locP2 = classifyOrientation(orientP2);
        if(locP2 == PtLineLocation::Right)
        {
            const VertInd iP1 = t.vertices[cw(i)];
            const T orientP1 = orient2D(vertices[iP1], a, b);
            const PtLineLocation::Enum locP1 = classifyOrientation(orientP1);
            if(locP1 == PtLineLocation::OnLine)
            {
                return make_tuple(noNeighbor, iP1, iP1);
            }
            if(locP1 == PtLineLocation::Left)
            {
                if(orientationTolerance)
                {
                    T closestOrient;
                    VertInd iClosestP;
                    if(std::abs(orientP1) <= std::abs(orientP2))
                    {
                        closestOrient = orientP1;
                        iClosestP = iP1;
                    }
                    else
                    {
                        closestOrient = orientP2;
                        iClosestP = iP2;
                    }
                    if(classifyOrientation(
                           closestOrient, orientationTolerance) ==
                       PtLineLocation::OnLine)
                    {
                        return make_tuple(noNeighbor, iClosestP, iClosestP);
                    }
                }
                return make_tuple(iT, iP1, iP2);
            }
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
    m_nearPtLocator.initialize(vertices);
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
std::vector<Edge>
Triangulation<T, TNearPointLocator>::insertVertex_FlipFixedEdges(
    const VertInd iVert)
{
    std::vector<Edge> flippedFixedEdges;

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

        if(isFlipNeeded(v, iVert, iV1, iV2, iV3))
        {
            // if flipped edge is fixed, remember it
            const Edge flippedEdge(iV1, iV3);
            if(fixedEdges.count(flippedEdge))
                flippedFixedEdges.push_back(flippedEdge);

            flipEdge(iT, iTopo);
            triStack.push(iT);
            triStack.push(iTopo);
        }
    }

    m_nearPtLocator.addPoint(iVert, vertices);
    return flippedFixedEdges;
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
    ensureDelaunayByEdgeFlips(v, iVert, triStack);
    m_nearPtLocator.addPoint(iVert, vertices);
}

template <typename T, typename TNearPointLocator>
void Triangulation<T, TNearPointLocator>::ensureDelaunayByEdgeFlips(
    const V2d<T>& v,
    const VertInd iVert,
    std::stack<TriInd>& triStack)
{
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
}

/*!
 * Handles super-triangle vertices.
 * Super-tri points are not infinitely far and influence the input points
 * Three cases are possible:
 *  1.  If one of the opposed vertices is super-tri: no flip needed
 *  2.  One of the shared vertices is super-tri:
 *      check if on point is same side of line formed by non-super-tri
 * vertices as the non-super-tri shared vertex
 *  3.  None of the vertices are super-tri: normal circumcircle test
 */
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
template <typename T, typename TNearPointLocator>
bool Triangulation<T, TNearPointLocator>::isFlipNeeded(
    const V2d<T>& v,
    const VertInd iV,
    const VertInd iV1,
    const VertInd iV2,
    const VertInd iV3) const
{
    const V2d<T>& v1 = vertices[iV1];
    const V2d<T>& v2 = vertices[iV2];
    const V2d<T>& v3 = vertices[iV3];
    if(m_superGeomType == SuperGeometryType::SuperTriangle)
    {
        // If flip-candidate edge touches super-triangle in-circumference
        // test has to be replaced with orient2d test against the line
        // formed by two non-artificial vertices (that don't belong to
        // super-triangle)
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

    // flip not needed if the original edge is fixed
    if(fixedEdges.count(Edge(iV1, iV3)))
        return false;

    return isFlipNeeded(v, iV, iV1, iV2, iV3);
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
    // make two new triangles and convert current triangle to 3rd new
    // triangle
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
    // only adjust adjacent triangles if triangulation is not finalized:
    // can happen when called from outside on an already finalized triangulation
    if(!isFinalized())
    {
        addAdjacentTriangle(v1, iTopo);
        addAdjacentTriangle(v3, iT);
        removeAdjacentTriangle(v2, iT);
        removeAdjacentTriangle(v4, iTopo);
    }
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

template <typename T, typename TNearPointLocator>
TriInd Triangulation<T, TNearPointLocator>::triangulatePseudopolygon(
    const VertInd ia,
    const VertInd ib,
    const std::vector<VertInd>::const_iterator pointsFirst,
    const std::vector<VertInd>::const_iterator pointsLast)
{
    if(pointsFirst == pointsLast)
        return pseudopolyOuterTriangle(ia, ib);
    // Find delaunay point
    const VertInd ic = findDelaunayPoint(ia, ib, pointsFirst, pointsLast);
    // Find pseudopolygons split by the delaunay point
    std::vector<VertInd>::const_iterator newLast = pointsFirst;
    while(*newLast != ic)
        ++newLast;
    const std::vector<VertInd>::const_iterator newFirst = newLast + 1;
    // triangulate splitted pseudo-polygons
    const TriInd iT2 = triangulatePseudopolygon(ic, ib, newFirst, pointsLast);
    const TriInd iT1 = triangulatePseudopolygon(ia, ic, pointsFirst, newLast);
    // add new triangle
    const Triangle t = {{ia, ib, ic}, {noNeighbor, iT2, iT1}};
    const TriInd iT = addTriangle(t);
    // adjust neighboring triangles and vertices
    if(iT1 != noNeighbor)
    {
        if(pointsFirst == newLast)
            changeNeighbor(iT1, ia, ic, iT);
        else
            triangles[iT1].neighbors[0] = iT;
    }
    if(iT2 != noNeighbor)
    {
        if(newFirst == pointsLast)
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
    const std::vector<VertInd>::const_iterator pointsFirst,
    const std::vector<VertInd>::const_iterator pointsLast) const
{
    assert(pointsFirst != pointsLast);
    const V2d<T>& a = vertices[ia];
    const V2d<T>& b = vertices[ib];
    VertInd ic = *pointsFirst;
    V2d<T> c = vertices[ic];
    typedef std::vector<VertInd>::const_iterator CIt;
    for(CIt it = pointsFirst + 1; it != pointsLast; ++it)
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

template <typename T, typename TNearPointLocator>
bool Triangulation<T, TNearPointLocator>::isFinalized() const
{
    return vertTris.empty() && !vertices.empty();
}

template <typename T, typename TNearPointLocator>
unordered_map<TriInd, LayerDepth>
Triangulation<T, TNearPointLocator>::peelLayer(
    std::stack<TriInd> seeds,
    const LayerDepth layerDepth,
    std::vector<LayerDepth>& triDepths) const
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

template <typename T, typename TNearPointLocator>
std::vector<LayerDepth>
Triangulation<T, TNearPointLocator>::calculateTriangleDepths() const
{
    std::vector<LayerDepth> triDepths(
        triangles.size(), std::numeric_limits<LayerDepth>::max());
    std::stack<TriInd> seeds(TriDeque(1, vertTris[0].front()));
    LayerDepth layerDepth = 0;
    LayerDepth deepestSeedDepth = 0;

    unordered_map<LayerDepth, TriIndUSet> seedsByDepth;
    do
    {
        const unordered_map<TriInd, LayerDepth>& newSeeds =
            peelLayer(seeds, layerDepth, triDepths);

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

} // namespace CDT
