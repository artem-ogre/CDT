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
#include <string>
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
        /**
         * Automatic insertion order optimized for better performance
         * @details breadth-first traversal of a Kd-tree for initial bulk-load,
         * randomized for subsequent insertions
         */
        Auto,
        /// insert vertices in same order they are provided
        AsProvided,
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
        NotAllowed, ///< constraint edge intersections are not allowed
        TryResolve, ///< attempt to resolve constraint edge intersections
        /**
         * No checks: slightly faster but less safe.
         * User must provide a valid input without intersecting constraints.
         */
        DontCheck,
    };
};

/**
 * Type used for storing layer depths for triangles
 * @note LayerDepth should support 60K+ layers, which could be to much or
 * too little for some use cases. Feel free to re-define this typedef.
 */
typedef unsigned short LayerDepth;
typedef LayerDepth BoundaryOverlapCount;

/**
 * Contains source location info: file, function, line
 */
class SourceLocation
{
public:
    /// Constructor
    SourceLocation(const std::string& file, const std::string& func, int line)
        : m_file(file)
        , m_func(func)
        , m_line(line)
    {}
    /// source file
    const std::string& file() const
    {
        return m_file;
    }
    /// source function
    const std::string& func() const
    {
        return m_func;
    }
    /// source line
    int line() const
    {
        return m_line;
    }

private:
    std::string m_file;
    std::string m_func;
    int m_line;
};

/// Macro for getting source location
#define CDT_SOURCE_LOCATION                                                    \
    SourceLocation(std::string(__FILE__), std::string(__func__), __LINE__)

/**
 * Base class for errors. Contains error description and source location: file,
 * function, line
 */
class Error : public std::runtime_error
{
public:
    /// Constructor
    Error(const std::string& description, const SourceLocation& srcLoc)
        : std::runtime_error(
              description + "\nin '" + srcLoc.func() + "' at " + srcLoc.file() +
              ":" + CDT::to_string(srcLoc.line()))
        , m_description(description)
        , m_srcLoc(srcLoc)
    {}
    /// Get error description
    const std::string& description() const
    {
        return m_description;
    }
    /// Get source location from where the error was thrown
    const SourceLocation& sourceLocation() const
    {
        return m_srcLoc;
    }

private:
    std::string m_description;
    SourceLocation m_srcLoc;
};

/**
 * Error thrown when triangulation modification is attempted after it was
 * finalized
 */
class FinalizedError : public Error
{
public:
    /// Constructor
    FinalizedError(const SourceLocation& srcLoc)
        : Error(
              "Triangulation was finalized with 'erase...' method. Further "
              "modification is not possible.",
              srcLoc)
    {}
};

/**
 * Error thrown when duplicate vertex is detected during vertex insertion
 */
class DuplicateVertexError : public Error
{
public:
    /// Constructor
    DuplicateVertexError(
        const VertInd v1,
        const VertInd v2,
        const SourceLocation& srcLoc)
        : Error(
              "Duplicate vertex detected: #" + CDT::to_string(v1) +
                  " is a duplicate of #" + CDT::to_string(v2),
              srcLoc)
        , m_v1(v1)
        , m_v2(v2)
    {}
    /// first duplicate
    VertInd v1() const
    {
        return m_v1;
    }
    /// second duplicate
    VertInd v2() const
    {
        return m_v2;
    }

private:
    VertInd m_v1, m_v2;
};

/**
 * Error thrown when intersecting constraint edges are detected, but
 * triangulation is not configured to attempt to resolve them
 */
class IntersectingConstraintsError : public Error
{
public:
    /// Constructor
    IntersectingConstraintsError(
        const Edge& e1,
        const Edge& e2,
        const SourceLocation& srcLoc)
        : Error(
              "Intersecting constraint edges detected: (" +
                  CDT::to_string(e1.v1()) + ", " + CDT::to_string(e1.v2()) +
                  ") intersects (" + CDT::to_string(e2.v1()) + ", " +
                  CDT::to_string(e2.v2()) + ")",
              srcLoc)
        , m_e1(e1)
        , m_e2(e2)
    {}
    /// first intersecting constraint
    const Edge& e1() const
    {
        return m_e1;
    }
    /// second intersecting constraint
    const Edge& e2() const
    {
        return m_e2;
    }

private:
    Edge m_e1, m_e2;
};

#ifdef CDT_ENABLE_CALLBACK_HANDLER

/**
 * What type of vertex is added to the triangulation
 */
struct CDT_EXPORT AddVertexType
{
    /**
     * The Enum itself
     * @note needed to pre c++11 compilers that don't support 'class enum'
     */
    enum Enum
    {
        /// Original vertex from user input
        UserInput,
        /// During conforming triangulation edge mid-point is added
        FixedEdgeMidpoint,
        /// Resolving fixed/constraint edges' intersection
        FixedEdgesIntersection,
    };
};

/**
 * What type of triangle change happened
 */
struct CDT_EXPORT TriangleChangeType
{
    /**
     * The Enum itself
     * @note needed to pre c++11 compilers that don't support 'class enum'
     */
    enum Enum
    {
        AddedNew,         ///< new triangle added to the triangulation
        ModifiedExisting, ///< existing triangle was modified
    };
};

// parameter names are used for documentation purposes, even if they are un-used
// in the interface's default implementation
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

/**
 * Interface for the callback handler that user can derive from and inject into
 * the triangulation to monitor certain events or order aborting the calculation
 */
class CDT_EXPORT ICallbackHandler
{
public:
    /// Virtual destructor
    virtual ~ICallbackHandler()
    {}

    /**
     *  Called when super-triangle is added.
     * 1.Added three new vertices with indices 0, 1, 2
     * 2. Added one a new triangle with an index 0
     */
    virtual void onAddSuperTriangle()
    {}

    /**
     * Called when inserted vertex is inside a triangle.
     * The triangle is split in three: original trianlge is re-purposed and two
     * new triangles are added
     * @param iRepurposedTri index of original triangle containing the point
     * that will be modified (re-purposed)
     * @param iNewTri1 index of first added new triangle
     * @param iNewTri2 index of second added new triangle
     */
    virtual void onInsertVertexInsideTriangle(
        const TriInd iRepurposedTri,
        const TriInd iNewTri1,
        const TriInd iNewTri2)
    {}

    /**
     * Called when inserted vertex is on an edge.
     * Two triangles sharing an edge are split in four:original triangles are
     * re-purposed and two new triangles are added.
     * @param iRepurposedTri1 index of first original triangle sharing an edge
     * that will be modified (re-purposed)
     * @param iRepurposedTri2 index of second original triangle sharing
     * an edge that will be modified (re-purposed)
     * @param iNewTri1 index of first added new triangle
     * @param iNewTri2 index of second added new triangle
     */
    virtual void onInsertVertexOnEdge(
        const TriInd iRepurposedTri1,
        const TriInd iRepurposedTri2,
        const TriInd iNewTri1,
        const TriInd iNewTri2)
    {}

    /**
     * Called just before an edge between tro triangles is flipped
     * @param iT index of the first triangle sharing an edge
     * @param iTopo index of the second triangle sharing an edge
     */
    virtual void onFlipEdge(const TriInd iT, const TriInd iTopo)
    {}

    /**
     * Called at the start of adding new vertex to the triangulation
     * @param iV index of the vertex
     * @param vertexType what type of vertex is being added
     */
    virtual void
    onAddVertexStart(const VertInd iV, const AddVertexType::Enum vertexType)
    {}

    /**
     * Called at the start of adding a constraint edge to the triangulation
     * @param edge constraint that is added/enforced
     */
    virtual void onAddEdgeStart(const Edge& edge)
    {}

    /**
     * Called when inserting a constraint edge causes polygon containing
     * triangles to be re-triangulated
     * @tris triangles in the polygon that will be re-triangulated and triangles
     * re-purposed
     */
    virtual void onReTriangulatePolygon(const std::vector<TriInd>& tris)
    {}

    /**
     * Tells whether the user wants to abort the triangulation at the earliest
     * opportunity
     */
    virtual bool isAbortCalculation() const
    {
        return false;
    };
};

// parameter names are used for documentation purposes, even if they are
// un-used in the interface's default implementation
#pragma GCC diagnostic pop

#endif

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
    explicit Triangulation(VertexInsertionOrder::Enum vertexInsertionOrder);
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
     * Insert constraint edges into triangulation for <b>Constrained Delaunay
     * Triangulation</b> (for example see figure below).
     *
     * Uses only original vertices: no new verties are added
     *
     * <img src="./images/show-case.png" alt="CDT show-case: constrained and
     * conforming triangulations, convex hulls, automatically removing holes"
     * style='height: 100%; width: 100%; max-height: 300px; object-fit:
     * contain'/>
     *
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
     * Insert constraint edges into triangulation for <b>Constrained Delaunay
     * Triangulation</b> (for example see figure below).
     *
     * Uses only original vertices: no new verties are added
     *
     * <img src="./images/show-case.png" alt="CDT show-case: constrained and
     * conforming triangulations, convex hulls, automatically removing holes"
     * style='height: 100%; width: 100%; max-height: 300px; object-fit:
     * contain'/>
     *
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
     * Insert constraint edges into triangulation for <b>Conforming Delaunay
     * Triangulation</b> (for example see figure below).
     *
     * May add new vertices.
     *
     * <img src="./images/show-case.png" alt="CDT show-case: constrained and
     * conforming triangulations, convex hulls, automatically removing holes"
     * style='height: 100%; width: 100%; max-height: 300px; object-fit:
     * contain'/>
     *
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
     * Insert constraint edges into triangulation for <b>Conforming Delaunay
     * Triangulation</b> (for example see figure below).
     *
     * May add new vertices.
     *
     * <img src="./images/show-case.png" alt="CDT show-case: constrained and
     * conforming triangulations, convex hulls, automatically removing holes"
     * style='height: 100%; width: 100%; max-height: 300px; object-fit:
     * contain'/>
     *
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

#ifdef CDT_ENABLE_CALLBACK_HANDLER
    /**
     * Set user-provided callback handler
     * @warning callback handler must outlive the triangulation class
     * @warning triangulatio does not own the callback and does not try to free
     * it
     * @param callbackHandler pointer to the callback handler
     */
    void setCallbackHandler(ICallbackHandler* callbackHandler);
#endif
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
     * Flip edge between two triangles given all the information such as
     * triangle vertices and neighbors
     */
    void flipEdge(
        TriInd iT,
        TriInd iTopo,
        VertInd v1,
        VertInd v2,
        VertInd v3,
        VertInd v4,
        TriInd n1,
        TriInd n2,
        TriInd n3,
        TriInd n4);

    /**
     * Remove triangles with specified indices.
     * Adjust internal triangulation state accordingly.
     * @param removedTriangles indices of triangles to remove
     */
    void removeTriangles(const TriIndUSet& removedTriangles);

    /// Access internal vertex adjacent triangles
    TriIndVec& VertTrisInternal();
    /// Access internal vertex adjacent triangles
    const TriIndVec& VertTrisInternal() const;
    /// @}

private:
    /*____ Detail __*/
    void addSuperTriangle(const Box2d<T>& box);
    void addNewVertex(const V2d<T>& pos, TriInd iT);
    void insertVertex(VertInd iVert);
    void insertVertex(VertInd iVert, VertInd walkStart);
    void ensureDelaunayByEdgeFlips(VertInd iV1, std::stack<TriInd>& triStack);
    /// Flip fixed edges and return a list of flipped fixed edges
    std::vector<Edge> insertVertex_FlipFixedEdges(VertInd iV1);

    /// State for an iteration of triangulate pseudo-polygon
    typedef tuple<IndexSizeType, IndexSizeType, TriInd, TriInd, Index>
        TriangulatePseudoPolygonTask;

    /**
     * Insert an edge into constraint Delaunay triangulation
     * @param edge edge to insert
     * @param originalEdge original edge inserted edge is part of
     * @param[in,out] remaining parts of the edge that still need to
     * be inserted
     * @param[in,out] tppIterations stack to be used for storing iterations of
     * triangulating pseudo-polygon
     * @note in-out state (@param remaining @param tppIterations) is shared
     * between different runs for performance gains (reducing memory
     * allocations)
     */
    void insertEdge(
        Edge edge,
        Edge originalEdge,
        EdgeVec& remaining,
        std::vector<TriangulatePseudoPolygonTask>& tppIterations);

    /**
     * Insert an edge or its part into constraint Delaunay triangulation
     * @param edge edge to insert
     * @param originalEdge original edge inserted edge is part of
     * @param[in,out] remainingStack parts of the edge that still need to
     * be inserted
     * @param[in,out] tppIterations stack to be used for storing iterations of
     * triangulating pseudo-polygon
     * @note in-out state (@param remaining @param tppIterations) is shared
     * between different runs for performance gains (reducing memory
     * allocations)
     */
    void insertEdgeIteration(
        Edge edge,
        Edge originalEdge,
        EdgeVec& remaining,
        std::vector<TriangulatePseudoPolygonTask>& tppIterations);

    /// State for iteration of conforming to edge
    typedef tuple<Edge, EdgeVec, BoundaryOverlapCount> ConformToEdgeTask;

    /**
     * Conform Delaunay triangulation to a fixed edge by recursively inserting
     * mid point of the edge and then conforming to its halves
     * @param edge fixed edge to conform to
     * @param originals original edges that new edge is piece of
     * @param overlaps count of overlapping boundaries at the edge. Only used
     * when re-introducing edge with overlaps > 0
     * @param[in,out] remaining remaining edge parts to be conformed to
     * @note in-out state (@param remaining @param reintroduce) is shared
     * between different runs for performance gains (reducing memory
     * allocations)
     */
    void conformToEdge(
        Edge edge,
        EdgeVec originals,
        BoundaryOverlapCount overlaps,
        std::vector<ConformToEdgeTask>& remaining);

    /**
     * Iteration of conform to fixed edge.
     * @param edge fixed edge to conform to
     * @param originals original edges that new edge is piece of
     * @param overlaps count of overlapping boundaries at the edge. Only used
     * when re-introducing edge with overlaps > 0
     * @param[in,out] remaining remaining edge parts
     * @note in-out state (@param remaining @param reintroduce) is shared
     * between different runs for performance gains (reducing memory
     * allocations)
     */
    void conformToEdgeIteration(
        Edge edge,
        const EdgeVec& originals,
        BoundaryOverlapCount overlaps,
        std::vector<ConformToEdgeTask>& remaining);

    tuple<TriInd, VertInd, VertInd> intersectedTriangle(
        VertInd iA,
        const V2d<T>& a,
        const V2d<T>& b,
        T orientationTolerance = T(0)) const;
    /// Returns indices of three resulting triangles
    std::stack<TriInd> insertVertexInsideTriangle(VertInd v, TriInd iT);
    /// Returns indices of four resulting triangles
    std::stack<TriInd> insertVertexOnEdge(
        VertInd v,
        TriInd iT1,
        TriInd iT2,
        const bool doHandleFixedSplitEdge = false);
    array<TriInd, 2> trianglesAt(const V2d<T>& pos) const;
    array<TriInd, 2>
    walkingSearchTrianglesAt(VertInd iV, VertInd startVertex) const;
    TriInd walkTriangles(VertInd startVertex, const V2d<T>& pos) const;
    /// Given triangle and its vertex find opposite triangle and the other three
    /// vertices and surrounding neighbors
    void edgeFlipInfo(
        TriInd iT,
        VertInd iV1,
        TriInd& iTopo,
        VertInd& iV2,
        VertInd& iV3,
        VertInd& iV4,
        TriInd& n1,
        TriInd& n2,
        TriInd& n3,
        TriInd& n4);
    bool isFlipNeeded(VertInd iV1, VertInd iV2, VertInd iV3, VertInd iV4) const;
    void changeNeighbor(TriInd iT, TriInd oldNeighbor, TriInd newNeighbor);
    void changeNeighbor(
        TriInd iT,
        VertInd iVedge1,
        VertInd iVedge2,
        TriInd newNeighbor);
    void triangulatePseudoPolygon(
        const std::vector<VertInd>& poly,
        unordered_map<Edge, TriInd>& outerTris,
        TriInd iT,
        TriInd iN,
        std::vector<TriInd>& trianglesToReuse,
        std::vector<TriangulatePseudoPolygonTask>& iterations);
    void triangulatePseudoPolygonIteration(
        const std::vector<VertInd>& poly,
        unordered_map<Edge, TriInd>& outerTris,
        std::vector<TriInd>& trianglesToReuse,
        std::vector<TriangulatePseudoPolygonTask>& iterations);
    IndexSizeType findDelaunayPoint(
        const std::vector<VertInd>& poly,
        IndexSizeType iA,
        IndexSizeType iB) const;
    TriInd addTriangle(const Triangle& t);
    TriInd addTriangle();
    /**
     * Remove super-triangle (if used) and triangles with specified indices.
     * Adjust internal triangulation state accordingly.
     * @removedTriangles indices of triangles to remove
     */
    void finalizeTriangulation(const TriIndUSet& removedTriangles);
    TriIndUSet growToBoundary(std::stack<TriInd> seeds) const;
    void fixEdge(const Edge& edge);
    void fixEdge(const Edge& edge, const Edge& originalEdge);
    /**
     *  Split existing constraint (fixed) edge
     * @param edge fixed edge to split
     * @param iSplitVert index of the vertex to be used as a split vertex
     */
    void splitFixedEdge(const Edge& edge, const VertInd iSplitVert);
    /**
     * Add a vertex that splits an edge into the triangulation
     * @param splitVert position of split vertex
     * @param iT index of a first triangle adjacent to the split edge
     * @param iTopo index of a second triangle adjacent to the split edge
     * (opposed to the first triangle)
     * @return index of a newly added split vertex
     */
    VertInd addSplitEdgeVertex(
        const V2d<T>& splitVert,
        const TriInd iT,
        const TriInd iTopo);
    /**
     * Split fixed edge and add a split vertex into the triangulation
     * @param edge fixed edge to split
     * @param splitVert position of split vertex
     * @param iT index of a first triangle adjacent to the split edge
     * @param iTopo index of a second triangle adjacent to the split edge
     * (opposed to the first triangle)
     * @return index of a newly added split vertex
     */
    VertInd splitFixedEdgeAt(
        const Edge& edge,
        const V2d<T>& splitVert,
        const TriInd iT,
        const TriInd iTopo);
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

    void insertVertices_AsProvided(VertInd superGeomVertCount);
    void insertVertices_Randomized(VertInd superGeomVertCount);
    void insertVertices_KDTreeBFS(VertInd superGeomVertCount, Box2d<T> box);
    std::pair<TriInd, TriInd> edgeTriangles(VertInd a, VertInd b) const;
    bool hasEdge(VertInd a, VertInd b) const;
    void setAdjacentTriangle(const VertInd v, const TriInd t);
    void pivotVertexTriangleCW(VertInd v);
    /// Add vertex to nearest-point locator if locator is initialized
    void tryAddVertexToLocator(const VertInd v);
    /// Perform lazy initialization of nearest-point locator after the Kd-tree
    /// BFS bulk load if necessary
    void tryInitNearestPointLocator();

    TNearPointLocator m_nearPtLocator;
    VertInd m_nTargetVerts;
    SuperGeometryType::Enum m_superGeomType;
    VertexInsertionOrder::Enum m_vertexInsertionOrder;
    IntersectingConstraintEdges::Enum m_intersectingEdgesStrategy;
    T m_minDistToConstraintEdge;
    TriIndVec m_vertTris; /// one triangle adjacent to each vertex
#ifdef CDT_ENABLE_CALLBACK_HANDLER
    ICallbackHandler* m_callbackHandler; /// user-provided callback handler
#endif
};

/// @}
/// @}

namespace detail
{

/// SplitMix64  pseudo-random number generator
struct SplitMix64RandGen
{
    typedef unsigned long long uint64; ///< uint64 type
    uint64 m_state;                    ///< PRNG's state
    /// constructor
    explicit SplitMix64RandGen(uint64 state)
        : m_state(state)
    {}
    /// default constructor
    explicit SplitMix64RandGen()
        : m_state(0)
    {}
    /// functor's operator
    uint64 operator()()
    {
        uint64 z = (m_state += 0x9e3779b97f4a7c15);
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9;
        z = (z ^ (z >> 27)) * 0x94d049bb133111eb;
        return z ^ (z >> 31);
    }
};

/// backport from c++11
template <class RandomIt>
void random_shuffle(RandomIt first, RandomIt last)
{
    detail::SplitMix64RandGen prng;
    typename std::iterator_traits<RandomIt>::difference_type i, n;
    n = last - first;
    for(i = n - 1; i > 0; --i)
    {
        std::swap(first[i], first[prng() % (i + 1)]);
    }
}

/// backport from c++11
template <class ForwardIt, class T>
void iota(ForwardIt first, ForwardIt last, T value)
{
    while(first != last)
    {
        *first++ = value;
        ++value;
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
        handleException(FinalizedError(CDT_SOURCE_LOCATION));

    const bool isFirstTime = vertices.empty();

    //
    // performance optimization: pre-allocate triangles and vertices
    //
    const std::size_t nNewVertices = std::distance(first, last);
    std::size_t exactCapacityTriangles = triangles.size() + 2 * nNewVertices;
    std::size_t exactCapacityVertices = vertices.size() + nNewVertices;
    if(isFirstTime) // account for adding super-triangle on the first run
    {
        exactCapacityTriangles += 1;
        exactCapacityVertices += nSuperTriangleVertices;
    }
    std::size_t capacityTriangles = exactCapacityTriangles;
    std::size_t capacityVertices = exactCapacityVertices;
    // to avoid re-allocation and unused memory
    // over-allocate by a fixed factor
    // when constraint edge intersections are resolved and vertices are many
    // because vertex is added for each intersection
    // and total number of intersections is unknown
    const VertInd overAllocationVerticesThreshold(1000);
    const T overAllocationFactor(1.1);
    const bool isOverPreAllocated =
        m_intersectingEdgesStrategy ==
            IntersectingConstraintEdges::TryResolve &&
        VertInd(nNewVertices) >= overAllocationVerticesThreshold;
    if(isOverPreAllocated)
    {
        capacityTriangles *= overAllocationFactor;
        capacityVertices *= overAllocationFactor;
    }
    triangles.reserve(capacityTriangles);
    vertices.reserve(capacityVertices);
    m_vertTris.reserve(capacityVertices);

    Box2d<T> box;
    if(isFirstTime)
    {
        box.envelopPoints(first, last, getX, getY);
        addSuperTriangle(box);
    }
    tryInitNearestPointLocator();
    const VertInd nExistingVerts = static_cast<VertInd>(vertices.size());

    for(TVertexIter it = first; it != last; ++it)
        addNewVertex(V2d<T>(getX(*it), getY(*it)), noNeighbor);

    switch(m_vertexInsertionOrder)
    {
    case VertexInsertionOrder::AsProvided:
        insertVertices_AsProvided(nExistingVerts);
        break;
    case VertexInsertionOrder::Auto:
        isFirstTime ? insertVertices_KDTreeBFS(nExistingVerts, box)
                    : insertVertices_Randomized(nExistingVerts);
        break;
    }

// make sure pre-allocation was correct
#ifdef CDT_ENABLE_CALLBACK_HANDLER
    assert(
        !m_callbackHandler || m_callbackHandler->isAbortCalculation() ||
        (vertices.size() == exactCapacityVertices));
    assert(
        !m_callbackHandler || m_callbackHandler->isAbortCalculation() ||
        (triangles.size() == exactCapacityTriangles));
#else
    assert(vertices.size() == exactCapacityVertices);
    assert(triangles.size() == exactCapacityTriangles);
#endif
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
        handleException(FinalizedError(CDT_SOURCE_LOCATION));

    std::vector<TriangulatePseudoPolygonTask> tppIterations;
    EdgeVec remaining;
    for(; first != last; ++first)
    {
#ifdef CDT_ENABLE_CALLBACK_HANDLER
        if(m_callbackHandler && m_callbackHandler->isAbortCalculation())
        {
            return;
        }
#endif
        // +3 to account for super-triangle vertices
        const Edge edge(
            VertInd(getStart(*first) + m_nTargetVerts),
            VertInd(getEnd(*first) + m_nTargetVerts));
        insertEdge(edge, edge, remaining, tppIterations);
    }
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
        handleException(FinalizedError(CDT_SOURCE_LOCATION));

    tryInitNearestPointLocator();
    // state shared between different runs for performance gains
    std::vector<ConformToEdgeTask> remaining;
    for(; first != last; ++first)
    {
#ifdef CDT_ENABLE_CALLBACK_HANDLER
        if(m_callbackHandler && m_callbackHandler->isAbortCalculation())
        {
            return;
        }
#endif
        // +3 to account for super-triangle vertices
        const Edge e(
            VertInd(getStart(*first) + m_nTargetVerts),
            VertInd(getEnd(*first) + m_nTargetVerts));
        conformToEdge(e, EdgeVec(1, e), 0, remaining);
    }
}

} // namespace CDT

#ifndef CDT_USE_AS_COMPILED_LIBRARY
#include "Triangulation.hpp"
#endif

#endif // header-guard
