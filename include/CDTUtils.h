/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#ifndef CDT_obwOaxOTdAWcLNTlNnaq
#define CDT_obwOaxOTdAWcLNTlNnaq

// #define CDT_USE_STRONG_TYPING // strong type checks on indices

#include "predicates.h" // robust predicates: orient, in-circle

// check if c++11 is supported
#if __cplusplus >= 201103L || (defined(_MSC_VER) && _MSC_VER >= 1900)
#define CDT_CXX11_IS_SUPPORTED
#elif !defined(__cplusplus) && !defined(_MSC_VER)
typedef char couldnt_parse_cxx_standard[-1];
#endif

#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

// use fall-backs for c++11 features
#ifdef CDT_CXX11_IS_SUPPORTED
#include <array>
#include <functional>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#else
#include <boost/array.hpp>
#include <boost/functional/hash.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
namespace std
{
using boost::array;
using boost::hash;
using boost::tie;
using boost::tuple;
using boost::make_tuple;
using boost::unordered_map;
using boost::unordered_set;
} // namespace std
#endif

#ifdef CDT_USE_STRONG_TYPING
#include <boost/serialization/strong_typedef.hpp>
#define CDT_TYPEDEF(typeWhat, typeAs) BOOST_STRONG_TYPEDEF(typeWhat, typeAs)
// note: hashes for strong-typedefs are specialized at the bottom
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
const static VertInd noVertex =
    VertInd(std::numeric_limits<std::size_t>::max());

/// 2D bounding box
template <typename T>
struct Box2d
{
    V2d<T> min; /// min box corner
    V2d<T> max; /// max box corner
};

typedef std::vector<TriInd> TriIndVec;
typedef std::array<VertInd, 3> VerticesArr3;
typedef std::array<TriInd, 3> NeighborsArr3;

/// Triangulation vertex
template <typename T>
struct Vertex
{
    V2d<T> pos;
    TriIndVec triangles;

    static Vertex make(const V2d<T>& pos, const TriInd iTriangle)
    {
        Vertex out = {pos, std::vector<TriInd>(1, iTriangle)};
        return out;
    }
    static Vertex makeInTriangle(
        const V2d<T>& pos,
        const TriInd iT1,
        const TriInd iT2,
        const TriInd iT3)
    {
        Vertex out;
        out.pos = pos;
        TriIndVec& vTris = out.triangles;
        vTris.reserve(3);
        vTris.push_back(iT1);
        vTris.push_back(iT2);
        vTris.push_back(iT3);
        return out;
    }
    static Vertex makeOnEdge(
        const V2d<T>& pos,
        const TriInd iT1,
        const TriInd iT2,
        const TriInd iT3,
        const TriInd iT4)
    {
        Vertex out;
        out.pos = pos;
        TriIndVec& vTris = out.triangles;
        vTris.reserve(4);
        vTris.push_back(iT1);
        vTris.push_back(iT2);
        vTris.push_back(iT3);
        vTris.push_back(iT4);
        return out;
    }
};

/// Edge connecting two vertices: vertex with smaller index is always first
/// \note: hash Edge is specialized at the bottom
struct Edge
{
    Edge(VertInd iV1, VertInd iV2)
        : m_vertices(
              iV1 < iV2 ? std::make_pair(iV1, iV2) : std::make_pair(iV2, iV1))
    {}
    bool operator==(const Edge& other) const
    {
        return m_vertices == other.m_vertices;
    }
    VertInd v1() const
    {
        return m_vertices.first;
    }
    VertInd v2() const
    {
        return m_vertices.second;
    }
    const std::pair<VertInd, VertInd>& verts() const
    {
        return m_vertices;
    }

private:
    std::pair<VertInd, VertInd> m_vertices;
};

typedef std::unordered_set<Edge> EdgeUSet;
typedef std::unordered_set<TriInd> TriIndUSet;
typedef std::unordered_map<TriInd, TriInd> TriIndUMap;

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
    VerticesArr3 vertices;
    NeighborsArr3 neighbors;
};

typedef std::vector<Triangle> TriangleVec;

/// Advance vertex or neighbor index counter-clockwise
inline Index ccw(Index i)
{
    return Index((i + 1) % 3);
}

/// Advance vertex or neighbor index clockwise
inline Index cw(Index i)
{
    return Index((i + 2) % 3);
}

/// Location of point on a triangle
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

inline bool isOnEdge(const PtTriLocation::Enum location)
{
    return location > PtTriLocation::Outside;
}

/// Neighbor index from a on-edge location
/// \note Call only if located on the edge!
inline Index edgeNeighbor(const PtTriLocation::Enum location)
{
    assert(location >= PtTriLocation::OnEdge1);
    return static_cast<Index>(location - PtTriLocation::OnEdge1);
}

/// Relative location of point to a line
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
    if(orientation == T(0))
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
    typedef typename std::vector<V2d<T> >::const_iterator Cit;
    for(Cit it = vertices.begin(); it != vertices.end(); ++it)
    {
        const V2d<T>& v = *it;
        box.min.x = std::min(v.x, box.min.x);
        box.max.x = std::max(v.x, box.max.x);
        box.min.y = std::min(v.y, box.min.y);
        box.max.y = std::max(v.y, box.max.y);
    }
    return box;
}

/// Opposed neighbor index from vertex index
inline Index opoNbr(const Index vertIndex)
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
inline Index opoVrt(const Index neighborIndex)
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
inline Index opposedTriangleInd(const Triangle& tri, const VertInd iVert)
{
    for(Index vi = Index(0); vi < Index(3); ++vi)
        if(iVert == tri.vertices[vi])
            return opoNbr(vi);
    throw std::runtime_error("Could not find opposed triangle index");
}

/// Index of triangle's neighbor opposed to an edge
inline Index opposedTriangleInd(
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
inline Index opposedVertexInd(const Triangle& tri, const TriInd iTopo)
{
    for(Index ni = Index(0); ni < Index(3); ++ni)
        if(iTopo == tri.neighbors[ni])
            return opoVrt(ni);
    throw std::runtime_error("Could not find opposed vertex index");
}

/// If triangle has a given neighbor return neighbor-index, throw otherwise
inline Index neighborInd(const Triangle& tri, const TriInd iTnbr)
{
    for(Index ni = Index(0); ni < Index(3); ++ni)
        if(iTnbr == tri.neighbors[ni])
            return ni;
    throw std::runtime_error("Could not find neighbor triangle index");
}

/// If triangle has a given vertex return vertex-index, throw otherwise
inline Index vertexInd(const Triangle& tri, const VertInd iV)
{
    for(Index i = Index(0); i < Index(3); ++i)
        if(iV == tri.vertices[i])
            return i;
    throw std::runtime_error("Could not find vertex index in triangle");
}

/// Given triangle and a vertex find opposed triangle
inline TriInd opposedTriangle(const Triangle& tri, const VertInd iVert)
{
    return tri.neighbors[opposedTriangleInd(tri, iVert)];
}

/// Given two triangles, return vertex of first triangle opposed to the second
inline VertInd opposedVertex(const Triangle& tri, const TriInd iTopo)
{
    return tri.vertices[opposedVertexInd(tri, iTopo)];
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

/// Test if two vertices share at least one common triangle
template <typename T>
bool verticesShareEdge(const Vertex<T>& a, const Vertex<T>& b)
{
    const std::vector<TriInd>& aTris = a.triangles;
    const std::vector<TriInd>& bTris = b.triangles;
    for(TriIndVec::const_iterator it = aTris.begin(); it != aTris.end(); ++it)
        if(std::find(bTris.begin(), bTris.end(), *it) != bTris.end())
            return true;
    return false;
}

/// Distance between two 2D points
template <typename T>
T distance(const V2d<T>& a, const V2d<T>& b)
{
    const T dx = b.x - a.x;
    const T dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace CDT

//*****************************************************************************
// Specialize hash functions
//*****************************************************************************
#ifdef CDT_CXX11_IS_SUPPORTED
namespace std
#else
namespace boost
#endif
{

#ifdef CDT_USE_STRONG_TYPING

template <>
struct hash<CDT::VertInd>
{
    std::size_t operator()(const CDT::VertInd& vi) const
    {
        return std::hash<std::size_t>()(vi.t);
    }
};

template <>
struct hash<CDT::TriInd>
{
    std::size_t operator()(const CDT::TriInd& vi) const
    {
        return std::hash<std::size_t>()(vi.t);
    }
};

#endif // CDT_USE_STRONG_TYPING

template <>
struct hash<CDT::Edge>
{
    std::size_t operator()(const CDT::Edge& e) const
    {
        return hashEdge(e);
    }

private:
    static void hashCombine(std::size_t& seed, const CDT::VertInd& key)
    {
        std::hash<CDT::VertInd> hasher;
        seed ^= hasher(key) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    static std::size_t hashEdge(const CDT::Edge& e)
    {
        const std::pair<CDT::VertInd, CDT::VertInd>& vv = e.verts();
        std::size_t seed1(0);
        hashCombine(seed1, vv.first);
        hashCombine(seed1, vv.second);
        std::size_t seed2(0);
        hashCombine(seed2, vv.second);
        hashCombine(seed2, vv.first);
        return std::min(seed1, seed2);
    }
};
} // namespace std/boost

#endif // header guard
