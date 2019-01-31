#pragma once

#include "predicates.h"

#include <boost/foreach.hpp>
#include <boost/serialization/strong_typedef.hpp>
#include <boost/tr1/array.hpp>
#include <boost/tr1/unordered_set.hpp>
#include <boost/assign/list_of.hpp>

#include <limits>
#include <stack>

namespace CDT
{

typedef unsigned char Index;
typedef std::size_t VertInd;
typedef std::size_t TriInd;
const static std::size_t noNeighbor = std::numeric_limits<std::size_t>::max();

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

template <typename T>
struct Box2d
{
    V2d<T> min; /// min box corner
    V2d<T> max; /// max box corner
};

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

// Advance vertex or neighbor index counter-clockwise
Index ccw(Index i)
{
    return (i + 1) % 3;
}

// Advance vertex or neighbor index clockwise
Index cw(Index i)
{
    return (i + 2) % 3;
}

template <typename T>
bool isInsideTriangle(
    const V2d<T>& p,
    const V2d<T>& v1,
    const V2d<T>& v2,
    const V2d<T>& v3)
{
    using namespace predicates::adaptive;
    // positive => left, negative => right, zero => on
    return orient2d(v1.raw(), v2.raw(), p.raw()) > T(0) &&
           orient2d(v2.raw(), v3.raw(), p.raw()) > T(0) &&
           orient2d(v3.raw(), v1.raw(), p.raw()) > T(0);
}

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

Index opoNbr(const Index vertIndex)
{
    switch(vertIndex)
    {
    case 0:
        return 1;
    case 1:
        return 2;
    case 2:
        return 0;
    }
    throw std::runtime_error("Invalid vertex index");
}

Index opoVrt(const Index neighborIndex)
{
    switch(neighborIndex)
    {
    case 0:
        return 2;
    case 1:
        return 0;
    case 2:
        return 1;
    }
    throw std::runtime_error("Invalid neighbor index");
}

Index opposedTriangleInd(const Triangle& tri, const VertInd iVert)
{
    for(Index vi = 0; vi < 3; ++vi)
        if(iVert == tri.vertices[vi])
            return opoNbr(vi);
    throw std::runtime_error("Could not find opposed triangle index");
}

Index opposedVertexInd(const Triangle& tri, const TriInd iTriOpo)
{
    for(Index ni = 0; ni < 3; ++ni)
        if(iTriOpo == tri.neighbors[ni])
            return opoVrt(ni);
    throw std::runtime_error("Could not find opposed vertex index");
}

Index neighborInd(const Triangle& tri, const TriInd iTriOpo)
{
    for(Index ni = 0; ni < 3; ++ni)
        if(iTriOpo == tri.neighbors[ni])
            return ni;
    throw std::runtime_error("Could not find neighbor triangle index");
}

TriInd opposedTriangle(const Triangle& tri, const VertInd iVert)
{
    return tri.neighbors[opposedTriangleInd(tri, iVert)];
}

VertInd opposedVertex(const Triangle& tri, const TriInd iTriOpo)
{
    return tri.vertices[opposedVertexInd(tri, iTriOpo)];
}

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
    insertPointInTriangle(const V2d<T>& pos, const TriInd iTri);
    TriInd triangleAt(const V2d<T>& pos) const;
    void flipEdge(const TriInd iTri, const TriInd iTriOpo);
    void changeNeighbor(
        const TriInd iTri,
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
    vertices.push_back(Vertex<T>::make(posV1, 0));
    vertices.push_back(Vertex<T>::make(posV2, 0));
    vertices.push_back(Vertex<T>::make(posV3, 0));
    Triangle superTri;
    for(Index i = 0; i < 3; ++i)
    {
        superTri.vertices[i] = i;
        superTri.neighbors[i] = noNeighbor;
    }
    triangles.push_back(superTri);
}

template <typename T>
void Triangulation<T>::insertVertex(const V2d<T>& pos)
{
    const VertInd iVert = vertices.size();
    std::stack<TriInd> triStack = insertPointInTriangle(pos, triangleAt(pos));
    while(!triStack.empty())
    {
        const TriInd iTri = triStack.top();
        triStack.pop();

        const Triangle& tri = triangles[iTri];
        const TriInd iTriOpo = opposedTriangle(tri, iVert);
        if(iTriOpo == noNeighbor)
            continue;
        const Triangle& triOpo = triangles[iTriOpo];
        const std::tr1::array<V2d<T>, 3> triOpoPts = {
            vertices[triOpo.vertices[0]].pos,
            vertices[triOpo.vertices[1]].pos,
            vertices[triOpo.vertices[2]].pos};
        if(isInCircumcircle(pos, triOpoPts))
        {
            flipEdge(iTri, iTriOpo);
            triStack.push(iTri);
            triStack.push(iTriOpo);
        }
    }
}

/* Insert point into triangle: split into 3 triangles:
 *  - create 2 new triangles
 *  - re-use old triangle for the 3rd
 *
 *                   v3
 *                  |
 *           new2-> | <-new1
 *                  v
 *          n3/    / \     \n2
 *                /   \
 *               / tri \
 *             v1_______v2
 *                 n1
 */
template <typename T>
std::stack<TriInd>
Triangulation<T>::insertPointInTriangle(const V2d<T>& pos, const TriInd iTri)
{
    Triangle& tri = triangles[iTri];
    const std::tr1::array<VertInd, 3> v = tri.vertices;
    const std::tr1::array<TriInd, 3> n = tri.neighbors;
    const VertInd iVert = vertices.size();
    const VertInd iNewTri1 = triangles.size();
    const VertInd iNewTri2 = iNewTri1 + 1;
    // make two new triangles
    const Triangle newTri1 = {{v[1], v[2], iVert}, {n[1], iNewTri2, iTri}};
    const Triangle newTri2 = {{v[2], v[0], iVert}, {n[2], iTri, iNewTri1}};
    // convert current triangle to third new triangle in-place
    tri.neighbors[1] = iNewTri1;
    tri.neighbors[2] = iNewTri2;
    tri.vertices[2] = iVert;
    // make new vertex
    Vertex<T> newVert = {pos, boost::assign::list_of(iTri)(iNewTri1)(iNewTri2)};
    // add new triangles and vertices to triangulation
    triangles.push_back(newTri1);
    triangles.push_back(newTri2);
    vertices.push_back(newVert);
    // adjust lists of adjacent triangles for v1, v2, v3
    addAdjacentTriangle(v[0],iNewTri2);
    addAdjacentTriangle(v[1],iNewTri1);
    removeAdjacentTriangle(v[2], iTri);
    addAdjacentTriangle(v[2],iNewTri1);
    addAdjacentTriangle(v[2],iNewTri2);
    // change triangle neighbor's neighbors to new triangles
    changeNeighbor(n[1], iTri, iNewTri1);
    changeNeighbor(n[2], iTri, iNewTri2);
    // return newly added triangles
    std::stack<TriInd> newTriangles;
    newTriangles.push(iTri);
    newTriangles.push(iNewTri1);
    newTriangles.push(iNewTri2);
    return newTriangles;
}

template <typename T>
TriInd Triangulation<T>::triangleAt(const V2d<T>& pos) const
{
    for(TriInd i = 0; i < triangles.size(); ++i)
    {
        const Triangle& tri = triangles[i];
        const V2d<T> v1 = vertices[tri.vertices[0]].pos;
        const V2d<T> v2 = vertices[tri.vertices[1]].pos;
        const V2d<T> v3 = vertices[tri.vertices[2]].pos;
        if(isInsideTriangle(pos, v1, v2, v3))
            return i;
    }
    throw std::runtime_error("no triangle was found");
}

/* Flip edge between T and Topo:
 *                * v1
 *               /'\
 *           n1 / ' \ n3
 *             /  '  \
 *         T->/   ' <--T'
 *           /    '    \
 *        v2 *===='====* v4
 *           \    '    /
 *       Topo'--> '   /<-Topo
 *             \  '  /
 *           n2 \ ' / n4
 *               \'/
 *                * v3
 */
template <typename T>
void Triangulation<T>::flipEdge(const TriInd iTri, const TriInd iTriOpo)
{
    Triangle& tri = triangles[iTri];
    Triangle& triOpo = triangles[iTriOpo];
    const std::tr1::array<TriInd, 3>& triNs = tri.neighbors;
    const std::tr1::array<TriInd, 3>& triOpoNs = triOpo.neighbors;
    const std::tr1::array<VertInd, 3>& triVs = tri.vertices;
    const std::tr1::array<VertInd, 3>& triOpoVs = triOpo.vertices;
    // find vertices and neighbors
    Index i = opposedVertexInd(tri, iTriOpo);
    const VertInd v1 = triVs[i];
    const VertInd v2 = triVs[ccw(i)];
    const TriInd n1 = triNs[i];
    const TriInd n3 = triNs[cw(i)];
    i = opposedVertexInd(triOpo, iTri);
    const VertInd v3 = triOpoVs[i];
    const VertInd v4 = triOpoVs[ccw(i)];
    const TriInd n4 = triOpoNs[i];
    const TriInd n2 = triOpoNs[cw(i)];
    // change vertices and neighbors
    tri = {{v4, v1, v3}, {n3, iTriOpo, n4}};
    triOpo = {{v2, v3, v1}, {n2, iTri, n1}};
    // adjust neighboring triangles and vertices
    changeNeighbor(n1, iTri, iTriOpo);
    changeNeighbor(n4, iTriOpo, iTri);
    addAdjacentTriangle(v1, iTriOpo);
    addAdjacentTriangle(v3, iTri);
    removeAdjacentTriangle(v2, iTri);
    removeAdjacentTriangle(v4, iTriOpo);
}

template <typename T>
void Triangulation<T>::changeNeighbor(
    const TriInd iTri,
    const TriInd oldNeighbor,
    const TriInd newNeighbor)
{
    if(iTri == noNeighbor)
        return;
    Triangle& tri = triangles[iTri];
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
