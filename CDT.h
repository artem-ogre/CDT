#pragma once

#include "predicates.h"

#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>
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
*      v3
*      /\
*   n3/  \n2
*    /____\
*  v1  n1  v2
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

struct PtInsideTri
{
    enum Enum
    {
        Inside,
        Ourside,
        OnTheEdge,
    };
};

/// Helper for inside-triangle test, checks one edge of triangle
template <typename T>
PtInsideTri::Enum
checkTriEdge(const V2d<T>& p, const V2d<T>& v1, const V2d<T>& v2)
{
    using namespace predicates::adaptive;
    const T orientation = orient2d(v1.raw(), v2.raw(), p.raw());
    if(orientation < T(0))
        return PtInsideTri::Ourside;
    else if(orientation == T(0))
        return PtInsideTri::OnTheEdge;
    return PtInsideTri::Inside;
}

/// Check if a point is inside a triangle defined by three points (2D)
template <typename T>
PtInsideTri::Enum isInsideTriangle(
    const V2d<T>& p,
    const V2d<T>& v1,
    const V2d<T>& v2,
    const V2d<T>& v3)
{
    using namespace predicates::adaptive;
    PtInsideTri::Enum result;
    result = checkTriEdge(p, v1, v2);
    if(result != PtInsideTri::Inside)
        return result;
    result = checkTriEdge(p, v2, v3);
    if(result != PtInsideTri::Inside)
        return result;
    return checkTriEdge(p, v3, v1);
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
bool isInCircumcircle(const V2d<T>& p, const std::tr1::array<V2d<T>, 3>& tri)
{
    using namespace predicates::adaptive;
    return incircle(tri[0].raw(), tri[1].raw(), tri[2].raw(), p.raw()) > T(0);
}

template <typename T>
class Triangulation
{
public:
    /*____ Data ____*/
    typedef std::pair<VertInd, VertInd> Edge;

    std::vector<Vertex<T> > vertices;
    std::vector<Triangle> triangles;
    std::tr1::unordered_set<Edge> fixedEdges;

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
    TriInd triangleAt(const V2d<T>& pos) const;
    void flipEdge(const TriInd iT, const TriInd iTopo);
    void changeNeighbor(
        const TriInd iT,
        const TriInd oldNeighbor,
        const TriInd newNeighbor);
    void addAdjacentTriangle(const VertInd iVertex, const TriInd iTriangle);
    void removeAdjacentTriangle(const VertInd iVertex, const TriInd iTriangle);
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
    std::stack<TriInd> triStack = insertPointInTriangle(pos, triangleAt(pos));
    while(!triStack.empty())
    {
        const TriInd iT = triStack.top();
        triStack.pop();

        const Triangle& t = triangles[iT];
        const TriInd iTopo = opposedTriangle(t, iVert);
        if(iTopo == noNeighbor)
            continue;
        const Triangle& tOpo = triangles[iTopo];
        const std::tr1::array<V2d<T>, 3> triOpoPts = {
            vertices[tOpo.vertices[0]].pos,
            vertices[tOpo.vertices[1]].pos,
            vertices[tOpo.vertices[2]].pos};
        if(isInCircumcircle(pos, triOpoPts))
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
    Vertex<T> newVert = {pos, boost::assign::list_of(iT)(iNewT1)(iNewT2)};
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

template <typename T>
TriInd Triangulation<T>::triangleAt(const V2d<T>& pos) const
{
    for(TriInd i = TriInd(0); i < TriInd(triangles.size()); ++i)
    {
        const Triangle& tri = triangles[i];
        const V2d<T> v1 = vertices[tri.vertices[0]].pos;
        const V2d<T> v2 = vertices[tri.vertices[1]].pos;
        const V2d<T> v3 = vertices[tri.vertices[2]].pos;
        if(isInsideTriangle(pos, v1, v2, v3) == PtInsideTri::Inside)
            return i;
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
void Triangulation<T>::changeNeighbor(const TriInd iT,
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
