/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#ifndef CDT_lNrmUayWQaIR5fxnsg9B
#define CDT_lNrmUayWQaIR5fxnsg9B

#include "CDTUtils.h"
#ifndef CDT_DONT_USE_BOOST_RTREE
#include "PointRTree.h"
#endif

#include <cassert>
#include <cstdlib>
#include <limits>
#include <stack>
#include <utility>
#include <vector>

namespace CDT
{

struct FindingClosestPoint
{
    enum Enum
    {
#ifndef CDT_DONT_USE_BOOST_RTREE
        BoostRTree,
#endif
        ClosestRandom,
    };
};

template <typename T>
class Triangulation
{
public:
    typedef std::vector<Vertex<T> > VertexVec;
    VertexVec vertices;
    TriangleVec triangles;
    EdgeUSet fixedEdges;

    /*____ API _____*/
    Triangulation(
        const FindingClosestPoint::Enum closestPtMode,
        const size_t nRandSamples = 10);
    void insertVertices(const std::vector<V2d<T> >& vertices);
    void insertEdges(const std::vector<Edge>& edges);
    void eraseSuperTriangle();
    void eraseOuterTriangles();

private:
    /*____ Detail __*/
    void addSuperTriangle(const Box2d<T>& box);
    void insertVertex(const V2d<T>& pos);
    void insertEdge(Edge edge);
    std::tuple<TriInd, VertInd, VertInd> intersectedTriangle(
        const VertInd iA,
        const std::vector<TriInd>& candidates,
        const V2d<T>& a,
        const V2d<T>& b) const;
    /// Returns indices of three resulting triangles
    std::stack<TriInd>
    insertPointInTriangle(const V2d<T>& pos, const TriInd iT);
    /// Returns indices of four resulting triangles
    std::stack<TriInd>
    insertPointOnEdge(const V2d<T>& pos, const TriInd iT1, const TriInd iT2);
    std::array<TriInd, 2> trianglesAt(const V2d<T>& pos) const;
    std::array<TriInd, 2> walkingSearchTrianglesAt(const V2d<T>& pos) const;
    VertInd
    nearestVertexRand(const V2d<T>& pos, const std::size_t nSamples) const;
#ifndef CDT_DONT_USE_BOOST_RTREE
    VertInd nearestVertexRtree(const V2d<T>& pos) const;
#endif
    bool isFlipNeeded(
        const V2d<T>& pos,
        const TriInd iT,
        const TriInd iTopo,
        const VertInd iVert) const;
    void flipEdge(const TriInd iT, const TriInd iTopo);
    void changeNeighbor(
        const TriInd iT,
        const TriInd oldNeighbor,
        const TriInd newNeighbor);
    void changeNeighbor(
        const TriInd iT,
        const VertInd iVedge1,
        const VertInd iVedge2,
        const TriInd newNeighbor);
    void addAdjacentTriangle(const VertInd iVertex, const TriInd iTriangle);
    void removeAdjacentTriangle(const VertInd iVertex, const TriInd iTriangle);
    TriInd triangulatePseudopolygon(
        const VertInd ia,
        const VertInd ib,
        const std::vector<VertInd>& points);
    VertInd findDelaunayPoint(
        const VertInd ia,
        const VertInd ib,
        const std::vector<VertInd>& points) const;
    TriInd pseudopolyOuterTriangle(const VertInd ia, const VertInd ib) const;
    TriInd addTriangle(const Triangle& t); // note: invalidates iterators!
    TriInd addTriangle(); // note: invalidates triangle iterators!
    void makeDummy(const TriInd iT);
    void eraseDummies();
    void eraseSuperTriangleVertices();

    std::vector<TriInd> m_dummyTris;
#ifndef CDT_DONT_USE_BOOST_RTREE
    PointRTree<T> m_rtree;
#endif
    std::size_t m_nRandSamples;
    FindingClosestPoint::Enum m_closestPtMode;
};

const static TriInd noNeighbor =
    TriInd(std::numeric_limits<std::size_t>::max());
const static VertInd noVertex =
    VertInd(std::numeric_limits<std::size_t>::max());

} // namespace CDT

#ifndef CDT_USE_AS_COMPILED_LIBRARY
#include "CDT.hpp"
#endif

#endif // header-guard
