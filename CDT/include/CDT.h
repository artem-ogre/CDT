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
#include "LocatorKDTree.h"
#include "remove_at.hpp"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <iterator>
#include <limits>
#include <memory>
#include <stack>
#include <utility>
#include <vector>

/// Namespace containing triangulation functionality
namespace CDT
{

/** @defgroup API Public API
 *  Contains API for constrained and conforming Delaunay triangulations
 */
/// @{

/**
 * Enum of strategies specifying order in which a range of vertices is inserted
 * @note VertexInsertionOrder::Randomized will only randomize order of
 * inserting in triangulation, vertex indices will be preserved as they were
 * specified in the final triangulation
 */
struct CDT_EXPORT VertexInsertionOrder
{
    /**
     * The Enum itself
     * @note needed to pre c++11 compilers that don't support 'class enum'
     */
    enum Enum
    {
        Randomized, ///< vertices will be inserted in random order
        AsProvided, ///< vertices will be inserted in the same order as provided
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
        Custom,        ///< user-specified custom geometry (e.g., grid)
    };
};

/**
 * Enum of strategies for treating intersecting constraint edges
 */
struct CDT_EXPORT IntersectingConstraintEdges
{
    /**
     * The Enum itself
     * @note needed to pre c++11 compilers that don't support 'class enum'
     */
    enum Enum
    {
        Ignore,  ///< constraint edge intersections are not checked
        Resolve, ///< constraint edge intersections are resolved
    };
};

/// Constant representing no valid neighbor for a triangle
const static TriInd noNeighbor(std::numeric_limits<TriInd>::max());
/// Constant representing no valid vertex for a triangle
const static VertInd noVertex(std::numeric_limits<VertInd>::max());

/**
 * Type used for storing layer depths for triangles
 * @note LayerDepth should support 60K+ layers, which could be to much or
 * too little for some use cases. Feel free to re-define this typedef.
 */
typedef unsigned short LayerDepth;
typedef LayerDepth BoundaryOverlapCount;

/// Triangles by vertex index
typedef std::vector<TriIndVec> VerticesTriangles;

/**
 * @defgroup Triangulation Triangulation Class
 * Class performing triangulations.
 */
/// @{

/**
 * Data structure representing a 2D constrained Delaunay triangulation
 *
 * @tparam T type of vertex coordinates (e.g., float, double)
 * @tparam TNearPointLocator class providing locating near point for efficiently
 * inserting new points. Provides methods: 'addPoint(vPos, iV)' and
 * 'nearPoint(vPos) -> iV'
 */
template <typename T, typename TNearPointLocator = LocatorKDTree<T> >
class CDT_EXPORT Triangulation
{
public:
    typedef std::vector<V2d<T> > V2dVec; ///< Vertices vector
    V2dVec vertices;                     ///< triangulation's vertices
    TriangleVec triangles;               ///< triangulation's triangles
    EdgeUSet fixedEdges; ///< triangulation's constraints (fixed edges)
    /**
     * triangles adjacent to each vertex
     * @note will be reset to empty when super-triangle is removed and
     * triangulation is finalized. To re-calculate adjacent triangles use
     * CDT::calculateTrianglesByVertex helper
     */
    VerticesTriangles vertTris;

    /** Stores count of overlapping boundaries for a fixed edge. If no entry is
     * present for an edge: no boundaries overlap.
     * @note map only has entries for fixed for edges that represent overlapping
     * boundaries
     * @note needed for handling depth calculations and hole-removel in case of
     * overlapping boundaries
     */
    unordered_map<Edge, BoundaryOverlapCount> overlapCount;

    /** Stores list of original edges represented by a given fixed edge
     * @note map only has entries for edges where multiple original fixed edges
     * overlap or where a fixed edge is a part of original edge created by
     * conforming Delaunay triangulation vertex insertion
     */
    unordered_map<Edge, EdgeVec> pieceToOriginals;

    /*____ API _____*/
    /// Default constructor
    Triangulation();
    /**
     * Constructor
     * @param vertexInsertionOrder strategy used for ordering vertex insertions
     */
    Triangulation(VertexInsertionOrder::Enum vertexInsertionOrder);
    /**
     * Constructor
     * @param vertexInsertionOrder strategy used for ordering vertex insertions
     * @param intersectingEdgesStrategy strategy for treating intersecting
     * constraint edges
     * @param minDistToConstraintEdge distance within which point is considered
     * to be lying on a constraint edge. Used when adding constraints to the
     * triangulation.
     */
    Triangulation(
        VertexInsertionOrder::Enum vertexInsertionOrder,
        IntersectingConstraintEdges::Enum intersectingEdgesStrategy,
        T minDistToConstraintEdge);
    /**
     * Constructor
     * @param vertexInsertionOrder strategy used for ordering vertex insertions
     * @param nearPtLocator class providing locating near point for efficiently
     * inserting new points
     * @param intersectingEdgesStrategy strategy for treating intersecting
     * constraint edges
     * @param minDistToConstraintEdge distance within which point is considered
     * to be lying on a constraint edge. Used when adding constraints to the
     * triangulation.
     */
    Triangulation(
        VertexInsertionOrder::Enum vertexInsertionOrder,
        const TNearPointLocator& nearPtLocator,
        IntersectingConstraintEdges::Enum intersectingEdgesStrategy,
        T minDistToConstraintEdge);
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
    /**
     * Insert vertices into triangulation
     * @param vertices vector of vertices to insert
     */
    void insertVertices(const std::vector<V2d<T> >& vertices);
    /**
     * Insert constraints (custom-type fixed edges) into triangulation
     * @note Each fixed edge is inserted by deleting the triangles it crosses,
     * followed by the triangulation of the polygons on each side of the edge.
     * <b> No new vertices are inserted.</b>
     * @note If some edge appears more than once in the input this means that
     * multiple boundaries overlap at the edge and impacts how hole detection
     * algorithm of Triangulation::eraseOuterTrianglesAndHoles works.
     * <b>Make sure there are no erroneous duplicates.</b>
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
    /**
     * Insert constraint edges into triangulation
     * @note Each fixed edge is inserted by deleting the triangles it crosses,
     * followed by the triangulation of the polygons on each side of the edge.
     * <b> No new vertices are inserted.</b>
     * @note If some edge appears more than once in the input this means that
     * multiple boundaries overlap at the edge and impacts how hole detection
     * algorithm of Triangulation::eraseOuterTrianglesAndHoles works.
     * <b>Make sure there are no erroneous duplicates.</b>
     * @tparam edges constraint edges
     */
    void insertEdges(const std::vector<Edge>& edges);
    /**
     * Ensure that triangulation conforms to constraints (fixed edges)
     * @note For each fixed edge that is not present in the triangulation its
     * midpoint is recursively added until the original edge is represented by a
     * sequence of its pieces. <b> New vertices are inserted.</b>
     * @note If some edge appears more than once the input this
     * means that multiple boundaries overlap at the edge and impacts how hole
     * detection algorithm of Triangulation::eraseOuterTrianglesAndHoles works.
     * <b>Make sure there are no erroneous duplicates.</b>
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
    void conformToEdges(
        TEdgeIter first,
        TEdgeIter last,
        TGetEdgeVertexStart getStart,
        TGetEdgeVertexEnd getEnd);
    /**
     * Ensure that triangulation conforms to constraints (fixed edges)
     * @note For each fixed edge that is not present in the triangulation its
     * midpoint is recursively added until the original edge is represented by a
     * sequence of its pieces. <b> New vertices are inserted.</b>
     * @note If some edge appears more than once the input this
     * means that multiple boundaries overlap at the edge and impacts how hole
     * detection algorithm of Triangulation::eraseOuterTrianglesAndHoles works.
     * <b>Make sure there are no erroneous duplicates.</b>
     * @tparam edges edges to conform to
     */
    void conformToEdges(const std::vector<Edge>& edges);
    /**
     * Erase triangles adjacent to super triangle
     *
     * @note does nothing if custom geometry is used
     */
    void eraseSuperTriangle();
    /// Erase triangles outside of constrained boundary using growing
    void eraseOuterTriangles();
    /**
     * Erase triangles outside of constrained boundary and auto-detected holes
     *
     * @note detecting holes relies on layer peeling based on layer depth
     * @note supports overlapping or touching boundaries
     */
    void eraseOuterTrianglesAndHoles();
    /**
     * Call this method after directly setting custom super-geometry via
     * vertices and triangles members
     */
    void initializedWithCustomSuperGeometry();

    /**
     * Check if the triangulation was finalized with `erase...` method and
     * super-triangle was removed.
     * @return true if triangulation is finalized, false otherwise
     */
    bool isFinalized() const;

    /**
     * @defgroup Advanced Advanced Triangulation Methods
     * Advanced methods for manually modifying the triangulation from
     * outside. Please only use them when you know what you are doing.
     */
    /// @{

    /**
     * Flip an edge between two triangle.
     * @note Advanced method for manually modifying the triangulation from
     * outside. Please call it when you know what you are doing.
     * @param iT first triangle
     * @param iTopo second triangle

     */
    void flipEdge(const TriInd iT, const TriInd iTopo);

    /**
     * Remove triangles with specified indices.
     * Adjust internal triangulation state accordingly.
     * @param removedTriangles indices of triangles to remove
     */
    void removeTriangles(const TriIndUSet& removedTriangles);
    /// @}

private:
    /*____ Detail __*/
    void addSuperTriangle(const Box2d<T>& box);
    void addNewVertex(const V2d<T>& pos, const TriIndVec& tris);
    void insertVertex(const VertInd iVert);
    void ensureDelaunayByEdgeFlips(
        const V2d<T>& v,
        const VertInd iVert,
        std::stack<TriInd>& triStack);
    /// Flip fixed edges and return a list of flipped fixed edges
    std::vector<Edge> insertVertex_FlipFixedEdges(const VertInd iVert);
    /**
     * Insert an edge into constraint Delaunay triangulation
     * @param edge edge to insert
     * @param originalEdge original edge inserted edge is part of
     */
    void insertEdge(Edge edge, Edge originalEdge);
    /**
     * Conform Delaunay triangulation to a fixed edge by recursively inserting
     * mid point of the edge and then conforming to its halves
     * @param edge fixed edge to conform to
     * @param originalEdges original edges that new edge is piece of
     * @param overlaps count of overlapping boundaries at the edge. Only used
     * when re-introducing edge with overlaps > 0
     * @param orientationTolerance tolerance for orient2d predicate,
     * values [-tolerance,+tolerance] are considered as 0.
     */
    void conformToEdge(
        Edge edge,
        EdgeVec originalEdges,
        BoundaryOverlapCount overlaps);
    tuple<TriInd, VertInd, VertInd> intersectedTriangle(
        const VertInd iA,
        const std::vector<TriInd>& candidates,
        const V2d<T>& a,
        const V2d<T>& b,
        T orientationTolerance = T(0)) const;
    /// Returns indices of three resulting triangles
    std::stack<TriInd> insertPointInTriangle(const VertInd v, const TriInd iT);
    /// Returns indices of four resulting triangles
    std::stack<TriInd>
    insertPointOnEdge(const VertInd v, const TriInd iT1, const TriInd iT2);
    array<TriInd, 2> trianglesAt(const V2d<T>& pos) const;
    array<TriInd, 2> walkingSearchTrianglesAt(const V2d<T>& pos) const;
    TriInd walkTriangles(const VertInd startVertex, const V2d<T>& pos) const;
    bool isFlipNeeded(
        const V2d<T>& v,
        const VertInd iV,
        const VertInd iV1,
        const VertInd iV2,
        const VertInd iV3) const;
    bool isFlipNeeded(
        const V2d<T>& v,
        const TriInd iT,
        const TriInd iTopo,
        const VertInd iVert) const;
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
    void addAdjacentTriangles(
        const VertInd iVertex,
        const TriInd iT1,
        const TriInd iT2,
        const TriInd iT3);
    void addAdjacentTriangles(
        const VertInd iVertex,
        const TriInd iT1,
        const TriInd iT2,
        const TriInd iT3,
        const TriInd iT4);
    void removeAdjacentTriangle(const VertInd iVertex, const TriInd iTriangle);
    TriInd triangulatePseudopolygon(
        const VertInd ia,
        const VertInd ib,
        const std::vector<VertInd>::const_iterator pointsFirst,
        const std::vector<VertInd>::const_iterator pointsLast);
    VertInd findDelaunayPoint(
        const VertInd ia,
        const VertInd ib,
        const std::vector<VertInd>::const_iterator pointsFirst,
        const std::vector<VertInd>::const_iterator pointsLast) const;
    TriInd pseudopolyOuterTriangle(const VertInd ia, const VertInd ib) const;
    TriInd addTriangle(const Triangle& t); // note: invalidates iterators!
    TriInd addTriangle(); // note: invalidates triangle iterators!
    /**
     * Remove super-triangle (if used) and triangles with specified indices.
     * Adjust internal triangulation state accordingly.
     * @removedTriangles indices of triangles to remove
     */
    void finalizeTriangulation(const TriIndUSet& removedTriangles);
    TriIndUSet growToBoundary(std::stack<TriInd> seeds) const;
    void fixEdge(const Edge& edge, const BoundaryOverlapCount overlaps);
    void fixEdge(const Edge& edge);
    void fixEdge(const Edge& edge, const Edge& originalEdge);
    /**
     * Flag triangle as dummy
     * @note Advanced method for manually modifying the triangulation from
     * outside. Please call it when you know what you are doing.
     * @param iT index of a triangle to flag
     */
    void makeDummy(const TriInd iT);
    /**
     * Erase all dummy triangles
     * @note Advanced method for manually modifying the triangulation from
     * outside. Please call it when you know what you are doing.
     */
    void eraseDummies();

    std::vector<TriInd> m_dummyTris;
    TNearPointLocator m_nearPtLocator;
    std::size_t m_nTargetVerts;
    SuperGeometryType::Enum m_superGeomType;
    VertexInsertionOrder::Enum m_vertexInsertionOrder;
    IntersectingConstraintEdges::Enum m_intersectingEdgesStrategy;
    T m_minDistToConstraintEdge;
};

/// @}

/** @defgroup helpers Helpers
 *  Helpers for working with CDT::Triangulation.
 */
/// @{

/**
 * Calculate triangles adjacent to vertices (triangles by vertex index)
 * @param triangles triangulation
 * @param verticesSize total number of vertices to pre-allocate the output
 * @return triangles by vertex index
 */
CDT_EXPORT VerticesTriangles
calculateTrianglesByVertex(const TriangleVec& triangles, VertInd verticesSize);

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
DuplicatesInfo FindDuplicates(
    TVertexIter first,
    TVertexIter last,
    TGetVertexCoordX getX,
    TGetVertexCoordY getY);

/**
 * Remove duplicates in-place from vector of custom points
 * @tparam TVertex vertex type
 * @tparam TAllocator allocator used by input vector of vertices
 * @param vertices vertices to remove duplicates from
 * @param duplicates information about duplicates
 */
template <typename TVertex, typename TAllocator>
void RemoveDuplicates(
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
 * @tparam TEdgeIter iterator that dereferences to custom edge type
 * @tparam TGetEdgeVertexStart function object getting start vertex index
 * from an edge.
 * Getter signature: const TEdgeIter::value_type& -> CDT::VertInd
 * @tparam TGetEdgeVertexEnd function object getting end vertex index from
 * an edge. Getter signature: const TEdgeIter::value_type& -> CDT::VertInd
 * @tparam TMakeEdgeFromStartAndEnd function object that makes new edge from
 * start and end vertices
 * @param first beginning of the range of edges
 * @param last end of the range of edges
 * @param mapping vertex-index mapping
 * @param getStart getter of edge start vertex index
 * @param getEnd getter of edge end vertex index
 * @param makeEdge factory for making edge from vetices
 */
template <
    typename TEdgeIter,
    typename TGetEdgeVertexStart,
    typename TGetEdgeVertexEnd,
    typename TMakeEdgeFromStartAndEnd>
CDT_EXPORT void RemapEdges(
    TEdgeIter first,
    TEdgeIter last,
    const std::vector<std::size_t>& mapping,
    TGetEdgeVertexStart getStart,
    TGetEdgeVertexEnd getEnd,
    TMakeEdgeFromStartAndEnd makeEdge);

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
 * @note Same as a chained call of CDT::FindDuplicates, CDT::RemoveDuplicates,
 * and CDT::RemapEdges
 * @tparam T type of vertex coordinates (e.g., float, double)
 * @tparam TVertex type of vertex
 * @tparam TGetVertexCoordX function object getting x coordinate from vertex.
 * Getter signature: const TVertexIter::value_type& -> T
 * @tparam TGetVertexCoordY function object getting y coordinate from vertex.
 * Getter signature: const TVertexIter::value_type& -> T
 * @tparam TEdgeIter iterator that dereferences to custom edge type
 * @tparam TGetEdgeVertexStart function object getting start vertex index
 * from an edge.
 * Getter signature: const TEdgeIter::value_type& -> CDT::VertInd
 * @tparam TGetEdgeVertexEnd function object getting end vertex index from
 * an edge. Getter signature: const TEdgeIter::value_type& -> CDT::VertInd
 * @tparam TMakeEdgeFromStartAndEnd function object that makes new edge from
 * start and end vertices
 * @param[in, out] vertices vertices to remove duplicates from
 * @param[in, out] edges collection of edges connecting vertices
 * @param getX getter of X-coordinate
 * @param getY getter of Y-coordinate
 * @param edgesFirst beginning of the range of edges
 * @param edgesLast end of the range of edges
 * @param getStart getter of edge start vertex index
 * @param getEnd getter of edge end vertex index
 * @param makeEdge factory for making edge from vetices
 * @returns information about vertex duplicates
 */
template <
    typename T,
    typename TVertex,
    typename TGetVertexCoordX,
    typename TGetVertexCoordY,
    typename TVertexAllocator,
    typename TEdgeIter,
    typename TGetEdgeVertexStart,
    typename TGetEdgeVertexEnd,
    typename TMakeEdgeFromStartAndEnd>
DuplicatesInfo RemoveDuplicatesAndRemapEdges(
    std::vector<TVertex, TVertexAllocator>& vertices,
    TGetVertexCoordX getX,
    TGetVertexCoordY getY,
    TEdgeIter edgesFirst,
    TEdgeIter edgesLast,
    TGetEdgeVertexStart getStart,
    TGetEdgeVertexEnd getEnd,
    TMakeEdgeFromStartAndEnd makeEdge);

/**
 * Same as a chained call of CDT::RemoveDuplicates + CDT::RemapEdges
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
 * then to next boundary and so on until all triangles are traversed.
 * For example depth is:
 *  - 0 for triangles outside outermost boundary
 *  - 1 for triangles inside boundary but outside hole
 *  - 2 for triangles in hole
 *  - 3 for triangles in island and so on...
 *
 * @param seed seed triangle to begin depth-peeling from
 * @param triangles triangles of triangulation
 * @param fixedEdges constraint edges of triangulation
 * @return vector where element at index i stores depth of i-th triangle
 */
CDT_EXPORT std::vector<LayerDepth> CalculateTriangleDepths(
    const TriInd seed,
    const TriangleVec& triangles,
    const EdgeUSet& fixedEdges);

/**
 * Calculate depth of each triangle in constraint triangulation. Supports
 * overlapping boundaries.
 *
 * Perform depth peeling from super triangle to outermost boundary,
 * then to next boundary and so on until all triangles are traversed.@n
 * For example depth is:
 *  - 0 for triangles outside outermost boundary
 *  - 1 for triangles inside boundary but outside hole
 *  - 2 for triangles in hole
 *  - 3 for triangles in island and so on...
 *
 * @param seed seed triangle to begin depth-peeling from
 * @param triangles triangles of triangulation
 * @param fixedEdges constraint edges of triangulation
 * @param overlapCount counts of boundary overlaps at fixed edges (map has
 * entries only for edges that represent overlapping boundaries)
 * @return vector where element at index i stores depth of i-th triangle
 */
CDT_EXPORT std::vector<LayerDepth> CalculateTriangleDepths(
    const TriInd seed,
    const TriangleVec& triangles,
    const EdgeUSet& fixedEdges,
    const unordered_map<Edge, BoundaryOverlapCount>& overlapCount);

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
 * @return triangles of the next (deeper) layer that are adjacent to the peeled
 * layer. To be used as seeds when peeling the next layer.
 */
CDT_EXPORT
TriIndUSet PeelLayer(
    std::stack<TriInd> seeds,
    const TriangleVec& triangles,
    const EdgeUSet& fixedEdges,
    const LayerDepth layerDepth,
    std::vector<LayerDepth>& triDepths);

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
 * @param overlapCount counts of boundary overlaps at fixed edges (map has
 * entries only for edges that represent overlapping boundaries)
 * @param layerDepth current layer's depth to mark triangles with
 * @param[in, out] triDepths depths of triangles
 * @return triangles of the deeper layers that are adjacent to the peeled layer.
 *         To be used as seeds when peeling deeper layers.
 */
CDT_EXPORT
unordered_map<TriInd, LayerDepth> PeelLayer(
    std::stack<TriInd> seeds,
    const TriangleVec& triangles,
    const EdgeUSet& fixedEdges,
    const unordered_map<Edge, BoundaryOverlapCount>& overlapCount,
    const LayerDepth layerDepth,
    std::vector<LayerDepth>& triDepths);

/**
 * Extract all edges of triangles
 *
 * @param triangles triangles used to extract edges
 * @return an unordered set of all edges of triangulation
 */
CDT_EXPORT EdgeUSet extractEdgesFromTriangles(const TriangleVec& triangles);

/*!
 * Converts piece->original_edges mapping to original_edge->pieces
 * @param pieceToOriginals maps pieces to original edges
 * @return mapping of original edges to pieces
 */
CDT_EXPORT unordered_map<Edge, EdgeVec>
EdgeToPiecesMapping(const unordered_map<Edge, EdgeVec>& pieceToOriginals);

/*!
 * Convert edge-to-pieces mapping into edge-to-split-vertices mapping
 * @tparam T type of vertex coordinates (e.g., float, double)
 * @param edgeToPieces edge-to-pieces mapping
 * @param vertices vertex buffer
 * @return mapping of edge-to-split-points.
 * Split points are sorted from edge's start (v1) to end (v2)
 */
template <typename T>
CDT_EXPORT unordered_map<Edge, std::vector<VertInd> > EdgeToSplitVertices(
    const unordered_map<Edge, EdgeVec>& edgeToPieces,
    const std::vector<V2d<T> >& vertices);

/// @}

/// @}

} // namespace CDT

//*****************************************************************************
// Implementations of template functionlity
//*****************************************************************************
// hash for CDT::V2d<T>
#ifdef CDT_CXX11_IS_SUPPORTED
namespace std
#else
namespace boost
#endif
{
template <typename T>
struct hash<CDT::V2d<T> >
{
    size_t operator()(const CDT::V2d<T>& xy) const
    {
#ifdef CDT_CXX11_IS_SUPPORTED
        typedef std::hash<T> Hasher;
#else
        typedef boost::hash<T> Hasher;
#endif
        return Hasher()(xy.x) ^ Hasher()(xy.y);
    }
};
} // namespace std

namespace CDT
{

namespace detail
{

static mt19937 randGenerator(9001);

template <class RandomIt>
void random_shuffle(RandomIt first, RandomIt last)
{
    typename std::iterator_traits<RandomIt>::difference_type i, n;
    n = last - first;
    for(i = n - 1; i > 0; --i)
    {
        std::swap(first[i], first[randGenerator() % (i + 1)]);
    }
}

} // namespace detail

//-----------------------
// Triangulation methods
//-----------------------
template <typename T, typename TNearPointLocator>
template <
    typename TVertexIter,
    typename TGetVertexCoordX,
    typename TGetVertexCoordY>
void Triangulation<T, TNearPointLocator>::insertVertices(
    const TVertexIter first,
    const TVertexIter last,
    TGetVertexCoordX getX,
    TGetVertexCoordY getY)
{
    if(isFinalized())
    {
        throw std::runtime_error(
            "Triangulation was finalized with 'erase...' method. Inserting new "
            "vertices is not possible");
    }
    detail::randGenerator.seed(9001); // ensure deterministic behavior
    if(vertices.empty())
    {
        addSuperTriangle(envelopBox<T>(first, last, getX, getY));
    }

    const std::size_t nExistingVerts = vertices.size();

    vertices.reserve(nExistingVerts + std::distance(first, last));
    for(TVertexIter it = first; it != last; ++it)
        addNewVertex(V2d<T>::make(getX(*it), getY(*it)), TriIndVec());

    switch(m_vertexInsertionOrder)
    {
    case VertexInsertionOrder::AsProvided:
        for(TVertexIter it = first; it != last; ++it)
            insertVertex(VertInd(nExistingVerts + std::distance(first, it)));
        break;
    case VertexInsertionOrder::Randomized:
        std::vector<VertInd> ii(std::distance(first, last));
        typedef std::vector<VertInd>::iterator Iter;
        VertInd value = static_cast<VertInd>(nExistingVerts);
        for(Iter it = ii.begin(); it != ii.end(); ++it, ++value)
            *it = value;
        detail::random_shuffle(ii.begin(), ii.end());
        for(Iter it = ii.begin(); it != ii.end(); ++it)
            insertVertex(*it);
        break;
    }
}

template <typename T, typename TNearPointLocator>
template <
    typename TEdgeIter,
    typename TGetEdgeVertexStart,
    typename TGetEdgeVertexEnd>
void Triangulation<T, TNearPointLocator>::insertEdges(
    TEdgeIter first,
    const TEdgeIter last,
    TGetEdgeVertexStart getStart,
    TGetEdgeVertexEnd getEnd)
{
    if(isFinalized())
    {
        throw std::runtime_error(
            "Triangulation was finalized with 'erase...' method. Inserting new "
            "edges is not possible");
    }
    for(; first != last; ++first)
    {
        // +3 to account for super-triangle vertices
        const Edge edge(
            VertInd(getStart(*first) + m_nTargetVerts),
            VertInd(getEnd(*first) + m_nTargetVerts));
        insertEdge(edge, edge);
    }
    eraseDummies();
}

template <typename T, typename TNearPointLocator>
template <
    typename TEdgeIter,
    typename TGetEdgeVertexStart,
    typename TGetEdgeVertexEnd>
void Triangulation<T, TNearPointLocator>::conformToEdges(
    TEdgeIter first,
    const TEdgeIter last,
    TGetEdgeVertexStart getStart,
    TGetEdgeVertexEnd getEnd)
{
    if(isFinalized())
    {
        throw std::runtime_error(
            "Triangulation was finalized with 'erase...' method. Conforming to "
            "new edges is not possible");
    }
    for(; first != last; ++first)
    {
        // +3 to account for super-triangle vertices
        const Edge e(
            VertInd(getStart(*first) + m_nTargetVerts),
            VertInd(getEnd(*first) + m_nTargetVerts));
        conformToEdge(e, EdgeVec(1, e), 0);
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
    typename TEdgeIter,
    typename TGetEdgeVertexStart,
    typename TGetEdgeVertexEnd,
    typename TMakeEdgeFromStartAndEnd>
void RemapEdges(
    TEdgeIter first,
    const TEdgeIter last,
    const std::vector<std::size_t>& mapping,
    TGetEdgeVertexStart getStart,
    TGetEdgeVertexEnd getEnd,
    TMakeEdgeFromStartAndEnd makeEdge)
{
    for(; first != last; ++first)
    {
        *first = makeEdge(
            static_cast<VertInd>(mapping[getStart(*first)]),
            static_cast<VertInd>(mapping[getEnd(*first)]));
    }
}

template <
    typename T,
    typename TVertex,
    typename TGetVertexCoordX,
    typename TGetVertexCoordY,
    typename TVertexAllocator,
    typename TEdgeIter,
    typename TGetEdgeVertexStart,
    typename TGetEdgeVertexEnd,
    typename TMakeEdgeFromStartAndEnd>
DuplicatesInfo RemoveDuplicatesAndRemapEdges(
    std::vector<TVertex, TVertexAllocator>& vertices,
    TGetVertexCoordX getX,
    TGetVertexCoordY getY,
    const TEdgeIter edgesFirst,
    const TEdgeIter edgesLast,
    TGetEdgeVertexStart getStart,
    TGetEdgeVertexEnd getEnd,
    TMakeEdgeFromStartAndEnd makeEdge)
{
    const DuplicatesInfo di =
        FindDuplicates<T>(vertices.begin(), vertices.end(), getX, getY);
    RemoveDuplicates(vertices, di.duplicates);
    RemapEdges(edgesFirst, edgesLast, di.mapping, getStart, getEnd, makeEdge);
    return di;
}

template <typename T>
unordered_map<Edge, std::vector<VertInd> > EdgeToSplitVertices(
    const unordered_map<Edge, EdgeVec>& edgeToPieces,
    const std::vector<V2d<T> >& vertices)
{
    typedef std::pair<VertInd, T> VertCoordPair;
    struct ComparePred
    {
        bool operator()(const VertCoordPair& a, const VertCoordPair& b) const
        {
            return a.second < b.second;
        }
    } comparePred;

    unordered_map<Edge, std::vector<VertInd> > edgeToSplitVerts;
    typedef unordered_map<Edge, EdgeVec>::const_iterator It;
    for(It it = edgeToPieces.begin(); it != edgeToPieces.end(); ++it)
    {
        const Edge& e = it->first;
        const T dX = vertices[e.v2()].x - vertices[e.v1()].x;
        const T dY = vertices[e.v2()].y - vertices[e.v1()].y;
        const bool isX = std::abs(dX) >= std::abs(dY); // X-coord longer
        const bool isAscending =
            isX ? dX >= 0 : dY >= 0; // Longer coordinate ascends
        const EdgeVec& pieces = it->second;
        std::vector<VertCoordPair> splitVerts;
        // size is:  2[ends] + (pieces - 1)[split vertices] = pieces + 1
        splitVerts.reserve(pieces.size() + 1);
        typedef EdgeVec::const_iterator EIt;
        for(EIt it = pieces.begin(); it != pieces.end(); ++it)
        {
            const array<VertInd, 2> vv = {it->v1(), it->v2()};
            typedef array<VertInd, 2>::const_iterator VIt;
            for(VIt v = vv.begin(); v != vv.end(); ++v)
            {
                const T c = isX ? vertices[*v].x : vertices[*v].y;
                splitVerts.push_back(std::make_pair(*v, isAscending ? c : -c));
            }
        }
        // sort by longest coordinate
        std::sort(splitVerts.begin(), splitVerts.end(), comparePred);
        // remove duplicates
        splitVerts.erase(
            std::unique(splitVerts.begin(), splitVerts.end()),
            splitVerts.end());
        assert(splitVerts.size() > 2); // 2 end points with split vertices
        std::pair<Edge, std::vector<VertInd> > val =
            std::make_pair(e, std::vector<VertInd>());
        val.second.reserve(splitVerts.size());
        typedef typename std::vector<VertCoordPair>::const_iterator SEIt;
        for(SEIt it = splitVerts.begin() + 1; it != splitVerts.end() - 1; ++it)
        {
            val.second.push_back(it->first);
        }
        edgeToSplitVerts.insert(val);
    }
    return edgeToSplitVerts;
}

} // namespace CDT

#ifndef CDT_USE_AS_COMPILED_LIBRARY
#include "CDT.hpp"
#endif

#endif // header-guard
