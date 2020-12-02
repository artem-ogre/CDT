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

#include "remove_at.hpp"

#include <cassert>
#include <cstdlib>
#include <iterator>
#include <limits>
#include <memory>
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

/// Enum of what type of geometry used to embed triangulation into
struct CDT_EXPORT SuperGeometryType
{
    /**
     * The Enum itself
     * @note needed to pre c++11 compilers that don't support 'class enum'
     */
    enum Enum
    {
        SuperTriangle, ///< conventional super-triangle
        Custom, ///< user-specified custom geometry (e.g., grid)
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
    /**
     * Insert custom point-types specified by iterator range and X/Y-getters
     * @tparam TVertexIter iterator that dereferences to custom point type
     * @tparam TGetVertexCoordX function object getting x coordinate from
     * vertex. Getter signature: const TVertexIter::value_type& -> T
     * @tparam TGetVertexCoordY function object getting y coordinate from
     * vertex. Getter signature: const TVertexIter::value_type& -> T
     * @param first beginning of the range of vertices to add
     * @param last end of the range of vertices to add
     * @param getX getter of X-coordinate
     * @param getY getter of Y-coordinate
     */
    template <
        typename TVertexIter,
        typename TGetVertexCoordX,
        typename TGetVertexCoordY>
    void insertVertices(
        TVertexIter first,
        TVertexIter last,
        TGetVertexCoordX getX,
        TGetVertexCoordY getY);
    /// Insert vertices into triangulation
    void insertVertices(const std::vector<V2d<T> >& vertices);
    /**
     * Insert constraints (custom-type fixed edges) into triangulation
     * @tparam TEdgeIter iterator that dereferences to custom edge type
     * @tparam TGetEdgeVertexStart function object getting start vertex index
     * from an edge.
     * Getter signature: const TEdgeIter::value_type& -> CDT::VertInd
     * @tparam TGetEdgeVertexEnd function object getting end vertex index from
     * an edge. Getter signature: const TEdgeIter::value_type& -> CDT::VertInd
     * @param first beginning of the range of edges to add
     * @param last end of the range of edges to add
     * @param getStart getter of edge start vertex index
     * @param getEnd getter of edge end vertex index
     */
    template <
        typename TEdgeIter,
        typename TGetEdgeVertexStart,
        typename TGetEdgeVertexEnd>
    void insertEdges(
        TEdgeIter first,
        TEdgeIter last,
        TGetEdgeVertexStart getStart,
        TGetEdgeVertexEnd getEnd);
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
    /**
     * Call this method after directly setting custom super-geometry via
     * vertices and triangles members
     */
    void initializedWithCustomSuperGeometry();

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
    std::size_t m_nTargetVerts;
    SuperGeometryType::Enum m_superGeomType;
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
 * Find duplicates in given custom point-type range
 * @note duplicates are points with exactly same X and Y coordinates
 * @tparam TVertexIter iterator that dereferences to custom point type
 * @tparam TGetVertexCoordX function object getting x coordinate from vertex.
 * Getter signature: const TVertexIter::value_type& -> T
 * @tparam TGetVertexCoordY function object getting y coordinate from vertex.
 * Getter signature: const TVertexIter::value_type& -> T
 * @param first beginning of the range of vertices
 * @param last end of the range of vertices
 * @param getX getter of X-coordinate
 * @param getY getter of Y-coordinate
 * @returns information about vertex duplicates
 */
template <
    typename T,
    typename TVertexIter,
    typename TGetVertexCoordX,
    typename TGetVertexCoordY>
CDT_EXPORT DuplicatesInfo FindDuplicates(
    TVertexIter first,
    TVertexIter last,
    TGetVertexCoordX getX,
    TGetVertexCoordY getY);

/**
 * Remove duplicates in-place from vector of custom points
 * @tparam TVertexIter iterator that dereferences to custom point type
 * @tparam TAllocator allocator used by input vector of vertices
 * @param vertices vertices to remove duplicates from
 * @param duplicates information about duplicates
 */
template <typename TVertex, typename TAllocator>
CDT_EXPORT void RemoveDuplicates(
    std::vector<TVertex, TAllocator>& vertices,
    const std::vector<std::size_t>& duplicates);

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
 * Find point duplicates, remove them from vector (in-place) and remap edges
 * (in-place)
 * @note Same as a chained call of @ref FindDuplicates, @ref RemoveDuplicates,
 * and @ref RemapEdges
 * @tparam TVertexIter iterator that dereferences to custom point type
 * @tparam TGetVertexCoordX function object getting x coordinate from vertex.
 * Getter signature: const TVertexIter::value_type& -> T
 * @tparam TGetVertexCoordY function object getting y coordinate from vertex.
 * Getter signature: const TVertexIter::value_type& -> T
 * @param[in, out] vertices vertices to remove duplicates from
 * @param[in, out] edges collection of edges connecting vertices
 * @param getX getter of X-coordinate
 * @param getY getter of Y-coordinate
 * @returns information about vertex duplicates
 */
template <
    typename T,
    typename TVertex,
    typename TGetVertexCoordX,
    typename TGetVertexCoordY,
    typename TVertexAllocator,
    typename TEdgeAllocator>
CDT_EXPORT DuplicatesInfo RemoveDuplicatesAndRemapEdges(
    std::vector<TVertex, TVertexAllocator>& vertices,
    std::vector<Edge, TEdgeAllocator>& edges,
    TGetVertexCoordX getX,
    TGetVertexCoordY getY);

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

//*****************************************************************************
// Implementations of template functionlity
//*****************************************************************************
// hash for CDT::V2d<T>
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

//-----------------------
// Triangulation methods
//-----------------------
template <typename T>
template <
    typename TVertexIter,
    typename TGetVertexCoordX,
    typename TGetVertexCoordY>
void Triangulation<T>::insertVertices(
    TVertexIter first,
    const TVertexIter last,
    TGetVertexCoordX getX,
    TGetVertexCoordY getY)
{
    if(vertices.empty())
        addSuperTriangle(envelopBox<T>(first, last, getX, getY));
    vertices.reserve(vertices.size() + std::distance(first, last));
    typedef typename std::vector<V2d<T> >::const_iterator Cit;
    for(; first != last; ++first)
        insertVertex(V2d<T>::make(getX(*first), getY(*first)));
}

template <typename T>
template <
    typename TEdgeIter,
    typename TGetEdgeVertexStart,
    typename TGetEdgeVertexEnd>
void Triangulation<T>::insertEdges(
    TEdgeIter first,
    const TEdgeIter last,
    TGetEdgeVertexStart getStart,
    TGetEdgeVertexEnd getEnd)
{
    for(; first != last; ++first)
    {
        // +3 to account for super-triangle vertices
        insertEdge(Edge(
            VertInd(getStart(*first) + m_nTargetVerts),
            VertInd(getEnd(*first) + m_nTargetVerts)));
    }
    eraseDummies();
}

//-----
// API
//-----
template <
    typename T,
    typename TVertexIter,
    typename TGetVertexCoordX,
    typename TGetVertexCoordY>
DuplicatesInfo FindDuplicates(
    TVertexIter first,
    TVertexIter last,
    TGetVertexCoordX getX,
    TGetVertexCoordY getY)
{
    typedef unordered_map<V2d<T>, std::size_t> PosToIndex;
    PosToIndex uniqueVerts;
    const std::size_t verticesSize = std::distance(first, last);
    DuplicatesInfo di = {
        std::vector<std::size_t>(verticesSize), std::vector<std::size_t>()};
    for(std::size_t iIn = 0, iOut = iIn; iIn < verticesSize; ++iIn, ++first)
    {
        typename PosToIndex::const_iterator it;
        bool isUnique;
        tie(it, isUnique) = uniqueVerts.insert(
            std::make_pair(V2d<T>::make(getX(*first), getY(*first)), iOut));
        if(isUnique)
        {
            di.mapping[iIn] = iOut++;
            continue;
        }
        di.mapping[iIn] = it->second; // found a duplicate
        di.duplicates.push_back(iIn);
    }
    return di;
}

template <typename TVertex, typename TAllocator>
void RemoveDuplicates(
    std::vector<TVertex, TAllocator>& vertices,
    const std::vector<std::size_t>& duplicates)
{
    vertices.erase(
        remove_at(
            vertices.begin(),
            vertices.end(),
            duplicates.begin(),
            duplicates.end()),
        vertices.end());
}

template <
    typename T,
    typename TVertex,
    typename TGetVertexCoordX,
    typename TGetVertexCoordY,
    typename TVertexAllocator,
    typename TEdgeAllocator>
DuplicatesInfo RemoveDuplicatesAndRemapEdges(
    std::vector<TVertex, TVertexAllocator>& vertices,
    std::vector<Edge, TEdgeAllocator>& edges,
    TGetVertexCoordX getX,
    TGetVertexCoordY getY)
{
    const DuplicatesInfo di =
        FindDuplicates<T>(vertices.begin(), vertices.end(), getX, getY);
    RemoveDuplicates(vertices, di.duplicates);
    RemapEdges(edges, di.mapping);
    return di;
}

} // namespace CDT

#ifndef CDT_USE_AS_COMPILED_LIBRARY
#include "CDT.hpp"
#endif

#endif // header-guard
