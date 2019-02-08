#pragma once

// #define CDT_USE_STRONG_TYPING // strong type checks on indices

#include "predicates.h"

#include <boost/foreach.hpp>
#ifdef CDT_USE_STRONG_TYPING
#include <boost/serialization/strong_typedef.hpp>
#endif
#include <boost/tr1/array.hpp>

#include <cassert>
#include <limits>
#include <vector>

#ifdef CDT_USE_STRONG_TYPING
#define CDT_TYPEDEF(typeWhat, typeAs) BOOST_STRONG_TYPEDEF(typeWhat, typeAs)
#else
#define CDT_TYPEDEF(typeWhat, typeAs) typedef typeWhat typeAs;
#endif

namespace CDT
{

/// 2D vector
template <typename T>
struct V2d
{
    T x;
    T y;

    const T* raw() const
    {
        return &x;
    }
    static V2d make(const T x, const T y)
    {
        V2d out = {x, y};
        return out;
    }
};

CDT_TYPEDEF(unsigned char, Index);
CDT_TYPEDEF(std::size_t, VertInd);
CDT_TYPEDEF(std::size_t, TriInd);
const static TriInd noNeighbor =
    TriInd(std::numeric_limits<std::size_t>::max());

/// 2D bounding box
template <typename T>
struct Box2d
{
    V2d<T> min; /// min box corner
    V2d<T> max; /// max box corner
};

/// Triangulation vertex
template <typename T>
struct Vertex
{
    V2d<T> pos;
    std::vector<TriInd> triangles;

    static Vertex make(const V2d<T>& pos, const TriInd iTriangle)
    {
        Vertex out = {pos, std::vector<TriInd>(1, iTriangle)};
        return out;
    }
};

/// Triangulation triangle
/* Counter-clockwise winding:
       v3
       /\
    n3/  \n2
     /____\
   v1  n1  v2
*/
struct Triangle
{
    std::tr1::array<VertInd, 3> vertices;
    std::tr1::array<TriInd, 3> neighbors;
};

/// Advance vertex or neighbor index counter-clockwise
Index ccw(Index i)
{
    return Index((i + 1) % 3);
}

/// Advance vertex or neighbor index clockwise
Index cw(Index i)
{
    return Index((i + 2) % 3);
}

struct PtTriLocation
{
    enum Enum
    {
        Inside,
        Outside,
        OnEdge1,
        OnEdge2,
        OnEdge3,
    };
};

bool isOnEdge(const PtTriLocation::Enum location)
{
    return location > PtTriLocation::Outside;
}

// Call only if located on the edge!
Index edgeNeighbor(const PtTriLocation::Enum location)
{
    assert(location >= PtTriLocation::OnEdge1);
    return static_cast<Index>(location - PtTriLocation::OnEdge1);
}

struct PtLineLocation
{
    enum Enum
    {
        Left,
        Right,
        OnLine,
    };
};

/// Check if point lies to the left of, to the right of, or on a line
template <typename T>
PtLineLocation::Enum
locatePointLine(const V2d<T>& p, const V2d<T>& v1, const V2d<T>& v2)
{
    using namespace predicates::adaptive;
    const T orientation = orient2d(v1.raw(), v2.raw(), p.raw());
    if(orientation < T(0))
        return PtLineLocation::Right;
    else if(orientation == T(0))
        return PtLineLocation::OnLine;
    return PtLineLocation::Left;
}

/// Check if point a lies inside of, outside of, or on an edge of a triangle
template <typename T>
PtTriLocation::Enum locatePointTriangle(
    const V2d<T>& p,
    const V2d<T>& v1,
    const V2d<T>& v2,
    const V2d<T>& v3)
{
    using namespace predicates::adaptive;
    PtTriLocation::Enum result = PtTriLocation::Inside;
    PtLineLocation::Enum edgeCheck = locatePointLine(p, v1, v2);
    if(edgeCheck == PtLineLocation::Right)
        return PtTriLocation::Outside;
    if(edgeCheck == PtLineLocation::OnLine)
        result = PtTriLocation::OnEdge1;
    edgeCheck = locatePointLine(p, v2, v3);
    if(edgeCheck == PtLineLocation::Right)
        return PtTriLocation::Outside;
    if(edgeCheck == PtLineLocation::OnLine)
        result = PtTriLocation::OnEdge2;
    edgeCheck = locatePointLine(p, v3, v1);
    if(edgeCheck == PtLineLocation::Right)
        return PtTriLocation::Outside;
    if(edgeCheck == PtLineLocation::OnLine)
        result = PtTriLocation::OnEdge3;
    return result;
}

/// Bounding box of a collection of 2D points
template <typename T>
Box2d<T> calculateBox(const std::vector<V2d<T> >& vertices)
{
    const T max = std::numeric_limits<T>::max();
    Box2d<T> box = {{max, max}, {-max, -max}};
    BOOST_FOREACH(const V2d<T>& v, vertices)
    {
        box.min.x = std::min(v.x, box.min.x);
        box.max.x = std::max(v.x, box.max.x);
        box.min.y = std::min(v.y, box.min.y);
        box.max.y = std::max(v.y, box.max.y);
    }
    return box;
}

/// Opposed neighbor index from vertex index
Index opoNbr(const Index vertIndex)
{
    if(vertIndex == Index(0))
        return Index(1);
    if(vertIndex == Index(1))
        return Index(2);
    if(vertIndex == Index(2))
        return Index(0);
    throw std::runtime_error("Invalid vertex index");
}

/// Opposed vertex index from neighbor index
Index opoVrt(const Index neighborIndex)
{
    if(neighborIndex == Index(0))
        return Index(2);
    if(neighborIndex == Index(1))
        return Index(0);
    if(neighborIndex == Index(2))
        return Index(1);
    throw std::runtime_error("Invalid neighbor index");
}

/// Index of triangle's neighbor opposed to a vertex
Index opposedTriangleInd(const Triangle& tri, const VertInd iVert)
{
    for(Index vi = Index(0); vi < Index(3); ++vi)
        if(iVert == tri.vertices[vi])
            return opoNbr(vi);
    throw std::runtime_error("Could not find opposed triangle index");
}

/// Index of triangle's neighbor opposed to an edge
Index opposedTriangleInd(
    const Triangle& tri,
    const VertInd iVedge1,
    const VertInd iVedge2)
{
    for(Index vi = Index(0); vi < Index(3); ++vi)
    {
        const VertInd iVert = tri.vertices[vi];
        if(iVert != iVedge1 && iVert != iVedge2)
            return opoNbr(vi);
    }
    throw std::runtime_error("Could not find opposed-to-edge triangle index");
}

/// Index of triangle's vertex opposed to a triangle
Index opposedVertexInd(const Triangle& tri, const TriInd iTopo)
{
    for(Index ni = Index(0); ni < Index(3); ++ni)
        if(iTopo == tri.neighbors[ni])
            return opoVrt(ni);
    throw std::runtime_error("Could not find opposed vertex index");
}

/// If triangle has a given neighbor return neighbor-index, throw otherwise
Index neighborInd(const Triangle& tri, const TriInd iTnbr)
{
    for(Index ni = Index(0); ni < Index(3); ++ni)
        if(iTnbr == tri.neighbors[ni])
            return ni;
    throw std::runtime_error("Could not find neighbor triangle index");
}

/// If triangle has a given vertex return vertex-index, throw otherwise
Index vertexInd(const Triangle& tri, const VertInd iV)
{
    for(Index i = Index(0); i < Index(3); ++i)
        if(iV == tri.vertices[i])
            return i;
    throw std::runtime_error("Could not find vertex index in triangle");
}

/// Given triangle and a vertex find opposed triangle
TriInd opposedTriangle(const Triangle& tri, const VertInd iVert)
{
    return tri.neighbors[opposedTriangleInd(tri, iVert)];
}

/// Given two triangles, return vertex of first triangle opposed to the second
VertInd opposedVertex(const Triangle& tri, const TriInd iTopo)
{
    return tri.vertices[opposedVertexInd(tri, iTopo)];
}

/// Test if triangle has vertex with a given index
bool hasVertex(const Triangle& tri, const VertInd iVert)
{
    BOOST_FOREACH(const VertInd triVert, tri.vertices)
        if(triVert == iVert)
            return true;
    return false;
}

/// Test if point lies in a circumscribed circle of a triangle
template <typename T>
bool isInCircumcircle(
    const V2d<T>& p,
    const V2d<T>& v1,
    const V2d<T>& v2,
    const V2d<T>& v3)
{
    using namespace predicates::adaptive;
    return incircle(v1.raw(), v2.raw(), v3.raw(), p.raw()) > T(0);
}

Triangle reverseTriangle(const Triangle& t)
{
    Triangle out = t;
    std::reverse(out.neighbors.begin(), out.neighbors.end());
    std::reverse(out.vertices.begin(), out.vertices.end());
    return out;
}

template <typename T>
bool verticesShareEdge(const Vertex<T>& a, const Vertex<T>& b)
{
    const std::vector<TriInd>& bTris = b.triangles;
    BOOST_FOREACH(const TriInd aTri, a.triangles)
        if(std::find(bTris.begin(), bTris.end(), aTri) != bTris.end())
            return true;
    return false;
}

typedef std::pair<VertInd, VertInd> Edge;
Edge makeEdge(const VertInd iV1, const VertInd iV2)
{
    return iV1 < iV2 ? std::make_pair(iV1, iV2) : std::make_pair(iV2, iV1);
}

} // namespace CDT
