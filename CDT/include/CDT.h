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

/// Constant representing no valid neighbor for a triangle
const static TriInd noNeighbor(std::numeric_limits<std::size_t>::max());
/// Constant representing no valid vertex for a triangle
const static VertInd noVertex(std::numeric_limits<std::size_t>::max());

/// Data structure representing a 2D triangulation
template <typename T>
class Triangulation
{
public:
    typedef std::vector<Vertex<T> > VertexVec; ///< Vertices vector
    VertexVec vertices;                        ///< triangulation's vertices
    TriangleVec triangles;                     ///< triangulation's triangles
    EdgeUSet fixedEdges; ///<  triangulation's constraints (fixed edges)

    /*____ API _____*/
    /// Constructor
    Triangulation(
        const FindingClosestPoint::Enum closestPtMode,
        const size_t nRandSamples = 10);
    /// Add vertices to triangulation
    void insertVertices(const std::vector<V2d<T> >& vertices);
    /// Add constraints (fixed edges) to triangulation
    void insertEdges(const std::vector<Edge>& edges);
    /// Erase triangles adjacent to super triangle
    void eraseSuperTriangle();
    /// Erase triangles outside of constrained boundary using growing
    void eraseOuterTriangles();
    /// Erase triangles outside of constrained boundary
    /// and automatically detected holes using growing
    void eraseOuterTrianglesAndHoles();

private:
    /*____ Detail __*/
    void addSuperTriangle(const Box2d<T>& box);
    void insertVertex(const V2d<T>& pos);
    void insertEdge(Edge edge);
    tuple<TriInd, VertInd, VertInd> intersectedTriangle(
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
    array<TriInd, 2> trianglesAt(const V2d<T>& pos) const;
    array<TriInd, 2> walkingSearchTrianglesAt(const V2d<T>& pos) const;
    TriInd walkTriangles(const VertInd startVertex, const V2d<T>& pos) const;
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
    template <typename TriIndexIter>
    void eraseTrianglesAtIndices(TriIndexIter first, TriIndexIter last);
    TriIndUSet growToBoundary(std::stack<TriInd> seeds) const;
    // return triangles behind boundary as second out parameter
    std::pair<TriIndUSet, TriIndUSet>
    growToBoundaryExt(std::stack<TriInd> seeds, TriIndUSet& traversed) const;

    std::vector<TriInd> m_dummyTris;
#ifndef CDT_DONT_USE_BOOST_RTREE
    PointRTree<T> m_rtree;
#endif
    std::size_t m_nRandSamples;
    FindingClosestPoint::Enum m_closestPtMode;
};

/// Removes duplicated points in-place, returns a mapping as a vector of indices
/// Example: vertices 0,1,3,4 where 0 and 3 are the same
/// will produce a vector [0,1,0,2] which are indices into [0,1,4]
template <typename T>
std::vector<std::size_t> RemoveDuplicates(std::vector<V2d<T> >& vertices);

/// Use given vertex index mapping to remap vertex indices in edges (in-place)
/// vertex mapping can be a result of RemoveDuplicates function
void RemapEdges(
    std::vector<Edge>& edges,
    const std::vector<std::size_t>& mapping);

/// Does the same as a chained call of RemoveDuplicates + RemapEdges
template <typename T>
std::vector<std::size_t> RemoveDuplicatesAndRemapEdges(
    std::vector<V2d<T> >& vertices,
    std::vector<Edge>& edges);

} // namespace CDT

#ifndef CDT_USE_AS_COMPILED_LIBRARY
#include "CDT.hpp"
#endif

#endif // header-guard
