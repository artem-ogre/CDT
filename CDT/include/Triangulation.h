/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/**
 * @file
 * Triangulation class
 */

#ifndef CDT_vW1vZ0lO8rS4gY4uI4fB
#define CDT_vW1vZ0lO8rS4gY4uI4fB

#include "CDTUtils.h"
#include "LocatorKDTree.h"

#include <algorithm>
#include <cstdlib>
#include <iterator>
#include <stack>
#include <stdexcept>
#include <utility>
#include <vector>

/// Namespace containing triangulation functionality
namespace CDT
{

/// @addtogroup API
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
     * @return vector where element at index i stores depth of i-th triangle
     */
    std::vector<LayerDepth> calculateTriangleDepths() const;

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
    void flipEdge(TriInd iT, TriInd iTopo);

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
    void insertVertex(VertInd iVert);
    void ensureDelaunayByEdgeFlips(
        const V2d<T>& v,
        VertInd iVert,
        std::stack<TriInd>& triStack);
    /// Flip fixed edges and return a list of flipped fixed edges
    std::vector<Edge> insertVertex_FlipFixedEdges(VertInd iVert);
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
        VertInd iA,
        const std::vector<TriInd>& candidates,
        const V2d<T>& a,
        const V2d<T>& b,
        T orientationTolerance = T(0)) const;
    /// Returns indices of three resulting triangles
    std::stack<TriInd> insertPointInTriangle(VertInd v, TriInd iT);
    /// Returns indices of four resulting triangles
    std::stack<TriInd> insertPointOnEdge(VertInd v, TriInd iT1, TriInd iT2);
    array<TriInd, 2> trianglesAt(const V2d<T>& pos) const;
    array<TriInd, 2> walkingSearchTrianglesAt(const V2d<T>& pos) const;
    TriInd walkTriangles(VertInd startVertex, const V2d<T>& pos) const;
    bool isFlipNeeded(
        const V2d<T>& v,
        VertInd iV,
        VertInd iV1,
        VertInd iV2,
        VertInd iV3) const;
    bool
    isFlipNeeded(const V2d<T>& v, TriInd iT, TriInd iTopo, VertInd iVert) const;
    void changeNeighbor(TriInd iT, TriInd oldNeighbor, TriInd newNeighbor);
    void changeNeighbor(
        TriInd iT,
        VertInd iVedge1,
        VertInd iVedge2,
        TriInd newNeighbor);
    void addAdjacentTriangle(VertInd iVertex, TriInd iTriangle);
    void
    addAdjacentTriangles(VertInd iVertex, TriInd iT1, TriInd iT2, TriInd iT3);
    void addAdjacentTriangles(
        VertInd iVertex,
        TriInd iT1,
        TriInd iT2,
        TriInd iT3,
        TriInd iT4);
    void removeAdjacentTriangle(VertInd iVertex, TriInd iTriangle);
    TriInd triangulatePseudopolygon(
        VertInd ia,
        VertInd ib,
        std::vector<VertInd>::const_iterator pointsFirst,
        std::vector<VertInd>::const_iterator pointsLast);
    VertInd findDelaunayPoint(
        VertInd ia,
        VertInd ib,
        std::vector<VertInd>::const_iterator pointsFirst,
        std::vector<VertInd>::const_iterator pointsLast) const;
    TriInd pseudopolyOuterTriangle(VertInd ia, VertInd ib) const;
    TriInd addTriangle(const Triangle& t); // note: invalidates iterators!
    TriInd addTriangle(); // note: invalidates triangle iterators!
    /**
     * Remove super-triangle (if used) and triangles with specified indices.
     * Adjust internal triangulation state accordingly.
     * @removedTriangles indices of triangles to remove
     */
    void finalizeTriangulation(const TriIndUSet& removedTriangles);
    TriIndUSet growToBoundary(std::stack<TriInd> seeds) const;
    void fixEdge(const Edge& edge, BoundaryOverlapCount overlaps);
    void fixEdge(const Edge& edge);
    void fixEdge(const Edge& edge, const Edge& originalEdge);
    /**
     * Flag triangle as dummy
     * @note Advanced method for manually modifying the triangulation from
     * outside. Please call it when you know what you are doing.
     * @param iT index of a triangle to flag
     */
    void makeDummy(TriInd iT);
    /**
     * Erase all dummy triangles
     * @note Advanced method for manually modifying the triangulation from
     * outside. Please call it when you know what you are doing.
     */
    void eraseDummies();
    /**
     * Depth-peel a layer in triangulation, used when calculating triangle
     * depths
     *
     * It takes starting seed triangles, traverses neighboring triangles, and
     * assigns given layer depth to the traversed triangles. Traversal is
     * blocked by constraint edges. Triangles behind constraint edges are
     * recorded as seeds of next layer and returned from the function.
     *
     * @param seeds indices of seed triangles
     * @param layerDepth current layer's depth to mark triangles with
     * @param[in, out] triDepths depths of triangles
     * @return triangles of the deeper layers that are adjacent to the peeled
     * layer. To be used as seeds when peeling deeper layers.
     */
    unordered_map<TriInd, LayerDepth> peelLayer(
        std::stack<TriInd> seeds,
        LayerDepth layerDepth,
        std::vector<LayerDepth>& triDepths) const;

    std::vector<TriInd> m_dummyTris;
    TNearPointLocator m_nearPtLocator;
    std::size_t m_nTargetVerts;
    SuperGeometryType::Enum m_superGeomType;
    VertexInsertionOrder::Enum m_vertexInsertionOrder;
    IntersectingConstraintEdges::Enum m_intersectingEdgesStrategy;
    T m_minDistToConstraintEdge;
};

/// @}
/// @}

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

} // namespace CDT

#ifndef CDT_USE_AS_COMPILED_LIBRARY
#include "Triangulation.hpp"
#endif

#endif // header-guard
