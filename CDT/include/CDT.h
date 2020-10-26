/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/**
 * @file
 * Public API
 */

#ifndef CDT_lNrmUayWQaIR5fxnsg9B
#define CDT_lNrmUayWQaIR5fxnsg9B

#include "CDTUtils.h"
#ifdef CDT_USE_BOOST
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

/// Enum of strategies for finding closest point to the newly inserted one
struct CDT_EXPORT FindingClosestPoint
{
    /**
     * The Enum itself
     * @note needed to pre c++11 compilers that don't support 'class enum'
     */
    enum Enum
    {
#ifdef CDT_USE_BOOST
        BoostRTree, ///< use boost::geometry::rtree
#endif
        ClosestRandom, ///< pick closest from few randomly selected candidates
    };
};

/// Constant representing no valid neighbor for a triangle
const static TriInd noNeighbor(std::numeric_limits<std::size_t>::max());
/// Constant representing no valid vertex for a triangle
const static VertInd noVertex(std::numeric_limits<std::size_t>::max());

/**
 * Data structure representing a 2D constrained Delaunay triangulation
 *
 * @tparam T type of vertex coordinates (e.g., float, double)
 */
template <typename T>
class CDT_EXPORT Triangulation
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
    /// Insert vertices into triangulation
    void insertVertices(const std::vector<V2d<T> >& vertices);
    /// Insert constraints (fixed edges) into triangulation
    void insertEdges(const std::vector<Edge>& edges);
    /// Erase triangles adjacent to super triangle
    void eraseSuperTriangle();
    /// Erase triangles outside of constrained boundary using growing
    void eraseOuterTriangles();
    /**
     * Erase triangles outside of constrained boundary and auto-detected holes
     *
     * @note detecting holes relies on layer peeling based on layer depth
     */
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
#ifdef CDT_USE_BOOST
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

    std::vector<TriInd> m_dummyTris;
#ifdef CDT_USE_BOOST
    PointRTree<T> m_rtree;
#endif
    std::size_t m_nRandSamples;
    FindingClosestPoint::Enum m_closestPtMode;
};

/**
 * Information about removed duplicated vertices.
 *
 * Contains mapping information and removed duplicates indices.
 * @note vertices {0,1,2,3,4} where 0 and 3 are the same will produce mapping
 *       {0,1,2,0,3} (to new vertices {0,1,2,3}) and duplicates {3}
 */
struct CDT_EXPORT DuplicatesInfo
{
    std::vector<std::size_t> mapping;    ///< vertex index mapping
    std::vector<std::size_t> duplicates; ///< duplicates' indices
};

/**
 * Remove duplicated points in-place
 *
 * @tparam T type of vertex coordinates (e.g., float, double)
 * @param[in, out] vertices collection of vertices to remove duplicates from
 * @returns information about duplicated vertices that were removed.
 */
template <typename T>
CDT_EXPORT DuplicatesInfo RemoveDuplicates(std::vector<V2d<T> >& vertices);

/**
 * Remap vertex indices in edges (in-place) using given vertex-index mapping.
 *
 * @note Mapping can be a result of RemoveDuplicates function
 * @param[in,out] edges collection of edges to remap
 * @param mapping vertex-index mapping
 */
CDT_EXPORT void
RemapEdges(std::vector<Edge>& edges, const std::vector<std::size_t>& mapping);

/**
 * Same as a chained call of @ref RemoveDuplicates + @ref RemapEdges
 *
 * @tparam T type of vertex coordinates (e.g., float, double)
 * @param[in, out] vertices collection of vertices to remove duplicates from
 * @param[in,out] edges collection of edges to remap
 */
template <typename T>
CDT_EXPORT DuplicatesInfo RemoveDuplicatesAndRemapEdges(
    std::vector<V2d<T> >& vertices,
    std::vector<Edge>& edges);

/**
 * Calculate depth of each triangle in constraint triangulation.
 *
 * Perform depth peeling from super triangle to outermost boundary,
 * then to next boundary and so on until all triangles are traversed.@n
 * For example depth is:
 *  - 0 for triangles outside outermost boundary
 *  - 1 for triangles inside boundary but outside hole
 *  - 2 for triangles in hole
 *  - 3 for triangles in island and so on...
 *
 * @tparam T type of vertex coordinates (e.g., float, double)
 * @param vertices vertices of triangulation
 * @param triangles triangles of triangulation
 * @param fixedEdges constraint edges of triangulation
 * @return vector where element at index i stores depth of i-th triangle
 */
template <typename T>
CDT_EXPORT std::vector<unsigned short> CalculateTriangleDepths(
    const std::vector<Vertex<T> >& vertices,
    const TriangleVec& triangles,
    const EdgeUSet& fixedEdges);

/**
 * Depth-peel a layer in triangulation, used when calculating triangle depths
 *
 * It takes starting seed triangles, traverses neighboring triangles, and
 * assigns given layer depth to the traversed triangles. Traversal is blocked by
 * constraint edges. Triangles behind constraint edges are recorded as seeds of
 * next layer and returned from the function.
 *
 * @tparam T type of vertex coordinates (e.g., float, double)
 * @param seeds indices of seed triangles
 * @param triangles triangles of triangulation
 * @param fixedEdges constraint edges of triangulation
 * @param layerDepth current layer's depth to mark triangles with
 * @param[in, out] triDepths depths of triangles
 * @return triangles of the next layer that are adjacent to the peeled layer.
 *         To be used as seeds when peeling the next layer.
 */
CDT_EXPORT
TriIndUSet PeelLayer(
    std::stack<TriInd> seeds,
    const TriangleVec& triangles,
    const EdgeUSet& fixedEdges,
    const unsigned short layerDepth,
    std::vector<unsigned short>& triDepths);

} // namespace CDT

#ifndef CDT_USE_AS_COMPILED_LIBRARY
#include "CDT.hpp"
#endif

#endif // header-guard
