/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#ifndef CDT_obwOaxOTdAWcLNTlNnaq
#define CDT_obwOaxOTdAWcLNTlNnaq

// #define CDT_USE_STRONG_TYPING // strong type checks on indices

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
using boost::make_tuple;
using boost::tie;
using boost::tuple;
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

    const T* raw() const;
    static V2d make(const T x, const T y);
};

CDT_TYPEDEF(unsigned char, Index);
CDT_TYPEDEF(std::size_t, VertInd);
CDT_TYPEDEF(std::size_t, TriInd);
typedef std::vector<TriInd> TriIndVec;
typedef std::array<VertInd, 3> VerticesArr3;
typedef std::array<TriInd, 3> NeighborsArr3;

/// 2D bounding box
template <typename T>
struct Box2d
{
    V2d<T> min; /// min box corner
    V2d<T> max; /// max box corner

    /// Bounding box of a collection of 2D points
    static Box2d envelop(const std::vector<V2d<T> >& vertices);
};

/// Triangulation vertex
template <typename T>
struct Vertex
{
    V2d<T> pos;
    TriIndVec triangles;

    static Vertex make(const V2d<T>& pos, const TriInd iTriangle);
    static Vertex makeInTriangle(
        const V2d<T>& pos,
        const TriInd iT1,
        const TriInd iT2,
        const TriInd iT3);
    static Vertex makeOnEdge(
        const V2d<T>& pos,
        const TriInd iT1,
        const TriInd iT2,
        const TriInd iT3,
        const TriInd iT4);
};

/// Edge connecting two vertices: vertex with smaller index is always first
/// \note: hash Edge is specialized at the bottom
struct Edge
{
    Edge(VertInd iV1, VertInd iV2);
    bool operator==(const Edge& other) const;
    VertInd v1() const;
    VertInd v2() const;
    const std::pair<VertInd, VertInd>& verts() const;

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
   v1  n1  v2                 */
struct Triangle
{
    VerticesArr3 vertices;
    NeighborsArr3 neighbors;
};

typedef std::vector<Triangle> TriangleVec;

/// Advance vertex or neighbor index counter-clockwise
Index ccw(Index i);

/// Advance vertex or neighbor index clockwise
Index cw(Index i);

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

/// Check if location is classified as on any of three edges
bool isOnEdge(const PtTriLocation::Enum location);

/// Neighbor index from a on-edge location
/// \note Call only if located on the edge!
Index edgeNeighbor(const PtTriLocation::Enum location);

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
locatePointLine(const V2d<T>& p, const V2d<T>& v1, const V2d<T>& v2);

/// Check if point a lies inside of, outside of, or on an edge of a triangle
template <typename T>
PtTriLocation::Enum locatePointTriangle(
    const V2d<T>& p,
    const V2d<T>& v1,
    const V2d<T>& v2,
    const V2d<T>& v3);

/// Opposed neighbor index from vertex index
inline Index opoNbr(const Index vertIndex);

/// Opposed vertex index from neighbor index
inline Index opoVrt(const Index neighborIndex);

/// Index of triangle's neighbor opposed to a vertex
inline Index opposedTriangleInd(const Triangle& tri, const VertInd iVert);

/// Index of triangle's neighbor opposed to an edge
inline Index opposedTriangleInd(
    const Triangle& tri,
    const VertInd iVedge1,
    const VertInd iVedge2);

/// Index of triangle's vertex opposed to a triangle
inline Index opposedVertexInd(const Triangle& tri, const TriInd iTopo);

/// If triangle has a given neighbor return neighbor-index, throw otherwise
inline Index neighborInd(const Triangle& tri, const TriInd iTnbr);

/// If triangle has a given vertex return vertex-index, throw otherwise
inline Index vertexInd(const Triangle& tri, const VertInd iV);

/// Given triangle and a vertex find opposed triangle
inline TriInd opposedTriangle(const Triangle& tri, const VertInd iVert);

/// Given two triangles, return vertex of first triangle opposed to the second
inline VertInd opposedVertex(const Triangle& tri, const TriInd iTopo);

/// Test if point lies in a circumscribed circle of a triangle
template <typename T>
bool isInCircumcircle(
    const V2d<T>& p,
    const V2d<T>& v1,
    const V2d<T>& v2,
    const V2d<T>& v3);

/// Test if two vertices share at least one common triangle
template <typename T>
bool verticesShareEdge(const Vertex<T>& a, const Vertex<T>& b);

/// Distance between two 2D points
template <typename T>
T distance(const V2d<T>& a, const V2d<T>& b);

} // namespace CDT

#ifndef CDT_USE_AS_COMPILED_LIBRARY
#include "CDTUtils.hpp"
#endif

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
