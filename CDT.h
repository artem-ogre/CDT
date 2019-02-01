#pragma once

#include "predicates.h"

#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>
#include <boost/functional/hash.hpp>
#include <boost/serialization/strong_typedef.hpp>
#include <boost/tr1/array.hpp>
#include <boost/tr1/unordered_set.hpp>

#include <limits>
#include <stack>

// #define CDT_USE_STRONG_TYPING // strong type checks on indices

#ifdef CDT_USE_STRONG_TYPING
#define CDT_TYPEDEF(typeWhat, typeAs) BOOST_STRONG_TYPEDEF(typeWhat, typeAs)
#else
#define CDT_TYPEDEF(typeWhat, typeAs) typedef typeWhat typeAs;
#endif

namespace CDT
{

CDT_TYPEDEF(unsigned char, Index);
CDT_TYPEDEF(std::size_t, VertInd);
CDT_TYPEDEF(std::size_t, TriInd);
const static std::size_t noNeighbor = std::numeric_limits<std::size_t>::max();

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

template <typename T>
class Triangulation
{
public:
    /*____ Data ____*/
    typedef std::pair<VertInd, VertInd> Edge;

    std::vector<Vertex<T> > vertices;
    std::vector<Triangle> triangles;
    std::tr1::unordered_set<Edge, boost::hash<Edge> > fixedEdges;

    /*____ API _____*/
    void insertVertices(const std::vector<V2d<T> >& vertices);
    void insertEdges(const std::vector<Edge>& edges);
    static Triangulation finalTriangulation();

private:
    /*____ Detail __*/
    void addSuperTriangle(const Box2d<T>& box);
    void insertVertex(const V2d<T>& pos);
    /// Returns indices of three resulting triangles
    std::stack<TriInd>
    insertPointInTriangle(const V2d<T>& pos, const TriInd iT);
    /// Returns indices of four resulting triangles
    std::stack<TriInd>
    insertPointOnEdge(const V2d<T>& pos, const TriInd iT1, const TriInd iT2);
    std::tr1::array<TriInd, 2> trianglesAt(const V2d<T>& pos) const;
    void flipEdge(const TriInd iT, const TriInd iTopo);
    void changeNeighbor(
        const TriInd iT,
        const TriInd oldNeighbor,
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
        const std::vector<VertInd>& points);
    TriInd pseudopolyOuterTriangle(const VertInd ia, const VertInd ib);
};

template <typename T>
void Triangulation<T>::addSuperTriangle(const Box2d<T>& box)
{
    const V2d<T> center = {box.min.x + box.max.x / T(2),
                           box.min.y + box.max.y / T(2)};
    const T w = box.max.x - box.min.x;
    const T h = box.max.y - box.min.y;
    const T diag = T(4) * std::sqrt(w * w + h * h);
    const T shift = diag / std::sqrt(2.0); // diagonal * sin(45deg)
    const V2d<T> posV1 = {center.x - shift, center.y - shift};
    const V2d<T> posV2 = {center.x + shift, center.y - shift};
    const V2d<T> posV3 = {center.x, center.y + diag};
    vertices.push_back(Vertex<T>::make(posV1, TriInd(0)));
    vertices.push_back(Vertex<T>::make(posV2, TriInd(0)));
    vertices.push_back(Vertex<T>::make(posV3, TriInd(0)));
    Triangle superTri;
    for(Index i = Index(0); i < Index(3); ++i)
    {
        superTri.vertices[i] = i;
        superTri.neighbors[i] = noNeighbor;
    }
    triangles.push_back(superTri);
}

template <typename T>
void Triangulation<T>::insertVertex(const V2d<T>& pos)
{
    const VertInd iVert(vertices.size());
    std::tr1::array<TriInd, 2> trisAt = trianglesAt(pos);
    std::stack<TriInd> triStack =
        trisAt[1] == noNeighbor ? insertPointInTriangle(pos, trisAt[0])
                                : insertPointOnEdge(pos, trisAt[0], trisAt[1]);
    while(!triStack.empty())
    {
        const TriInd iT = triStack.top();
        triStack.pop();

        const Triangle& t = triangles[iT];
        const TriInd iTopo = opposedTriangle(t, iVert);
        if(iTopo == noNeighbor)
            continue;
        const Triangle& tOpo = triangles[iTopo];
        const V2d<T>& v1 = vertices[tOpo.vertices[0]].pos;
        const V2d<T>& v2 = vertices[tOpo.vertices[1]].pos;
        const V2d<T>& v3 = vertices[tOpo.vertices[2]].pos;
        if(isInCircumcircle(pos, v1, v2, v3))
        {
            flipEdge(iT, iTopo);
            triStack.push(iT);
            triStack.push(iTopo);
        }
    }
}

/* Insert point into triangle: split into 3 triangles:
 *  - create 2 new triangles
 *  - re-use old triangle for the 3rd
 *                      v3
 *                    / | \
 *                   /  |  \
 *                  /   |   \
 *              n3 /    |    \ n2
 *                /new2 | new1\
 *               /      v      \
 *              /    __/ \__    \
 *             /  __/       \__  \
 *            / _/     tri     \_ \
 *          v1 ___________________ v2
 *                     n1
 */
template <typename T>
std::stack<TriInd>
Triangulation<T>::insertPointInTriangle(const V2d<T>& pos, const TriInd iT)
{
    Triangle& tri = triangles[iT];
    const std::tr1::array<VertInd, 3> vv = tri.vertices;
    const std::tr1::array<TriInd, 3> nn = tri.neighbors;
    const VertInd v1 = vv[0], v2 = vv[1], v3 = vv[2];
    const TriInd n1 = nn[0], n2 = nn[1], n3 = nn[2];
    const VertInd v(vertices.size());
    const TriInd iNewT1(triangles.size());
    const TriInd iNewT2(iNewT1 + 1);
    // make two new triangles and convert current triangle to 3rd new triangle
    const Triangle newTri1 = {{v2, v3, v}, {n2, iNewT2, iT}};
    const Triangle newTri2 = {{v3, v1, v}, {n3, iT, iNewT1}};
    tri = {{v1, v2, v}, {n1, iNewT1, iNewT2}};
    // make new vertex
    const Vertex<T> newVert = {pos, boost::assign::list_of(iT)(iNewT1)(iNewT2)};
    // add new triangles and vertices to triangulation
    triangles.push_back(newTri1);
    triangles.push_back(newTri2);
    vertices.push_back(newVert);
    // adjust lists of adjacent triangles for v1, v2, v3
    addAdjacentTriangle(v1, iNewT2);
    addAdjacentTriangle(v2, iNewT1);
    removeAdjacentTriangle(v3, iT);
    addAdjacentTriangle(v3, iNewT1);
    addAdjacentTriangle(v3, iNewT2);
    // change triangle neighbor's neighbors to new triangles
    changeNeighbor(n2, iT, iNewT1);
    changeNeighbor(n3, iT, iNewT2);
    // return newly added triangles
    std::stack<TriInd> newTriangles;
    newTriangles.push(iT);
    newTriangles.push(iNewT1);
    newTriangles.push(iNewT2);
    return newTriangles;
}

/* Inserting a point on the edge between two triangles
 *    T1 (top)        v1
 *                   /|\
 *              n1 /  |  \ n4
 *               /    |    \
 *             /  T1' | Tnew1\
 *           v2-------v- -----v4
 *             \ Tnew2| T2'  /
 *               \    |    /
 *              n2 \  |  / n3
 *                   \|/
 *   T2 (bottom)      v3
 */
template <typename T>
std::stack<TriInd> Triangulation<T>::insertPointOnEdge(
    const V2d<T>& pos,
    const TriInd iT1,
    const TriInd iT2)
{
    const VertInd v(vertices.size());
    const TriInd iTnew1(triangles.size());
    const TriInd iTnew2(iTnew1 + 1);

    Triangle& t1 = triangles[iT1];
    Triangle& t2 = triangles[iT2];
    Index i = opposedVertexInd(t1, iT2);
    const VertInd v1 = t1.vertices[i];
    const VertInd v2 = t1.vertices[ccw(i)];
    const TriInd n1 = t1.neighbors[i];
    const TriInd n4 = t1.neighbors[cw(i)];
    i = opposedVertexInd(t2, iT1);
    const VertInd v3 = t2.vertices[i];
    const VertInd v4 = t2.vertices[ccw(i)];
    const TriInd n3 = t2.neighbors[i];
    const TriInd n2 = t2.neighbors[cw(i)];
    // add new triangles and change existing ones
    t1 = {{v1, v2, v}, {n1, iTnew2, iTnew1}};
    t2 = {{v3, v4, v}, {n3, iTnew1, iTnew2}};
    const Triangle tNew1 = {{v1, v, v4}, {iT1, iT2, n4}};
    const Triangle tNew2 = {{v3, v, v2}, {iT2, iT1, n2}};
    // make new vertex
    const Vertex<T> vNew = {pos,
                            boost::assign::list_of(iT1)(iTnew2)(iT2)(iTnew1)};
    // adjust neighboring triangles and vertices
    changeNeighbor(n4, iT1, iTnew1);
    changeNeighbor(n2, iT2, iTnew2);
    addAdjacentTriangle(v1, iTnew1);
    addAdjacentTriangle(v3, iTnew2);
    removeAdjacentTriangle(v2, iT2);
    addAdjacentTriangle(v2, iTnew2);
    removeAdjacentTriangle(v4, iT1);
    addAdjacentTriangle(v4, iTnew1);
    // add new triangles and vertices to triangulation
    triangles.push_back(tNew1);
    triangles.push_back(tNew2);
    vertices.push_back(vNew);
    // return newly added triangles
    std::stack<TriInd> newTriangles;
    newTriangles.push(iT1);
    newTriangles.push(iTnew2);
    newTriangles.push(iT2);
    newTriangles.push(iTnew1);
    return newTriangles;
}

template <typename T>
std::tr1::array<TriInd, 2>
Triangulation<T>::trianglesAt(const V2d<T>& pos) const
{
    std::tr1::array<TriInd, 2> out = {noNeighbor, noNeighbor};
    for(TriInd i = TriInd(0); i < TriInd(triangles.size()); ++i)
    {
        const Triangle& tri = triangles[i];
        const V2d<T> v1 = vertices[tri.vertices[0]].pos;
        const V2d<T> v2 = vertices[tri.vertices[1]].pos;
        const V2d<T> v3 = vertices[tri.vertices[2]].pos;
        const PtTriLocation::Enum loc = locatePointTriangle(pos, v1, v2, v3);
        if(loc == PtTriLocation::Outside)
            continue;
        out[0] = i;
        if(isOnEdge(loc))
            out[1] = tri.neighbors[edgeNeighbor(loc)];
        return out;
    }
    throw std::runtime_error("no triangle was found");
}

/* Flip edge between T and Topo:
 *
 *                v4         | - old edge
 *               /|\         ~ - new edge
 *              / | \
 *          n3 /  T' \ n4
 *            /   |   \
 *           /    |    \
 *     T -> v1~~~~~~~~~v3 <- Topo
 *           \    |    /
 *            \   |   /
 *          n1 \Topo'/ n2
 *              \ | /
 *               \|/
 *                v2
 */
template <typename T>
void Triangulation<T>::flipEdge(const TriInd iT, const TriInd iTopo)
{
    Triangle& tri = triangles[iT];
    Triangle& triOpo = triangles[iTopo];
    const std::tr1::array<TriInd, 3>& triNs = tri.neighbors;
    const std::tr1::array<TriInd, 3>& triOpoNs = triOpo.neighbors;
    const std::tr1::array<VertInd, 3>& triVs = tri.vertices;
    const std::tr1::array<VertInd, 3>& triOpoVs = triOpo.vertices;
    // find vertices and neighbors
    Index i = opposedVertexInd(tri, iTopo);
    const VertInd v1 = triVs[i];
    const VertInd v2 = triVs[ccw(i)];
    const TriInd n1 = triNs[i];
    const TriInd n3 = triNs[cw(i)];
    i = opposedVertexInd(triOpo, iT);
    const VertInd v3 = triOpoVs[i];
    const VertInd v4 = triOpoVs[ccw(i)];
    const TriInd n4 = triOpoNs[i];
    const TriInd n2 = triOpoNs[cw(i)];
    // change vertices and neighbors
    tri = {{v4, v1, v3}, {n3, iTopo, n4}};
    triOpo = {{v2, v3, v1}, {n2, iT, n1}};
    // adjust neighboring triangles and vertices
    changeNeighbor(n1, iT, iTopo);
    changeNeighbor(n4, iTopo, iT);
    addAdjacentTriangle(v1, iTopo);
    addAdjacentTriangle(v3, iT);
    removeAdjacentTriangle(v2, iT);
    removeAdjacentTriangle(v4, iTopo);
}

template <typename T>
void Triangulation<T>::changeNeighbor(
    const TriInd iT,
    const TriInd oldNeighbor,
    const TriInd newNeighbor)
{
    if(iT == noNeighbor)
        return;
    Triangle& tri = triangles[iT];
    tri.neighbors[neighborInd(tri, oldNeighbor)] = newNeighbor;
}

template <typename T>
void Triangulation<T>::addAdjacentTriangle(
    const VertInd iVertex,
    const TriInd iTriangle)
{
    vertices[iVertex].triangles.push_back(iTriangle);
}

template <typename T>
void Triangulation<T>::removeAdjacentTriangle(
    const VertInd iVertex,
    const TriInd iTriangle)
{
    std::vector<TriInd>& tris = vertices[iVertex].triangles;
    tris.erase(std::remove(tris.begin(), tris.end(), iTriangle));
}

std::pair<std::vector<VertInd>, std::vector<VertInd> >
splitPseudopolygon(const VertInd vi, const std::vector<VertInd>& points)
{
    std::pair<std::vector<VertInd>, std::vector<VertInd> > out;
    std::vector<VertInd>::const_iterator it;
    for(it = points.begin(); vi != *it; ++it)
        out.first.push_back(*it);
    for(it = it + 1; it != points.end(); ++it)
        out.second.push_back(*it);
    return out;
}

template <typename T>
TriInd Triangulation<T>::triangulatePseudopolygon(
    const VertInd ia,
    const VertInd ib,
    const std::vector<VertInd>& points)
{
    if(points.empty())
        return pseudopolyOuterTriangle(ia, ib);
    const VertInd ic = findDelaunayPoint(ia, ib, points);
    const std::pair<std::vector<VertInd>, std::vector<VertInd> > splitted =
        splitPseudopolygon(ic, points);
    // triangulate splitted pseudo-polygons
    TriInd iT2 = triangulatePseudopolygon(ic, ib, splitted.second);
    TriInd iT1 = triangulatePseudopolygon(ia, ic, splitted.first);
    // add new triangle
    const TriInd iT = triangles.size();
    const Triangle t = {{ia, ib, ic}, {noNeighbor, iT2, iT1}};
    triangles.push_back(t);
    // adjust neighboring triangles and vertices
    triangles[iT1].neighbors[0] = iT;
    triangles[iT2].neighbors[0] = iT;
    addAdjacentTriangle(ia, iT);
    addAdjacentTriangle(ib, iT);
    addAdjacentTriangle(ic, iT);

    return iT;
}

template <typename T>
VertInd Triangulation<T>::findDelaunayPoint(
    const VertInd ia,
    const VertInd ib,
    const std::vector<VertInd>& points)
{
    assert(!points.empty());
    const V2d<T>& a = vertices[ia].pos;
    const V2d<T>& b = vertices[ib].pos;
    VertInd ic = points.front();
    V2d<T> c = vertices[ic].pos;
    typedef std::vector<VertInd>::const_iterator CIt;
    for(CIt it = points.begin() + 1; it != points.end(); ++it)
    {
        const V2d<T> v = vertices[*it].pos;
        if(!isInCircumcircle(v, a, b, c))
            continue;
        ic = *it;
        c = vertices[ic].pos;
    }
    return ic;
}

template <typename T>
TriInd
Triangulation<T>::pseudopolyOuterTriangle(const VertInd ia, const VertInd ib)
{
    const Vertex<T>& a = vertices[ia];
    const Vertex<T>& b = vertices[ib];
    BOOST_FOREACH(const TriInd iTa, a.triangles)
        BOOST_FOREACH(const TriInd iTb, b.triangles)
            if(iTa == iTb)
                return iTa;
    throw std::runtime_error(
        "Could not find an outer triangle of the pseudo-polygon edge");
}

template <typename T>
void Triangulation<T>::insertVertices(const std::vector<V2d<T> >& newVertices)
{
    if(vertices.empty())
        addSuperTriangle(calculateBox(newVertices));
    vertices.reserve(vertices.size() + newVertices.size());
    BOOST_FOREACH(const V2d<T>& v, newVertices)
        insertVertex(v);
}

} // namespace CDT
