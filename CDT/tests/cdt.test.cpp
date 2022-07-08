#include <CDT.h>
#include <VerifyTopology.h>

#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace CDT;

using CoordTypes = std::tuple<float, double>;
template <class CoordType>
using Vertices = std::vector<V2d<CoordType> >;

#define ENHANCED_THROW(ExceptionType, message)                                 \
    throw ExceptionType(                                                       \
        std::string(message) + " in " + __FILE__ + ':' +                       \
        std::to_string(__LINE__) + ':' + __func__)

namespace CDT
{

bool operator<(const Edge& lhs, const Edge& rhs)
{
    return lhs.v1() != rhs.v1() ? lhs.v1() < rhs.v1() : lhs.v2() < rhs.v2();
}

bool operator<(const Triangle& lhs, const Triangle& rhs)
{
    for(Index i = 0; i < Index(3); ++i)
    {
        if(lhs.vertices[i] == rhs.vertices[i])
            continue;
        return lhs.vertices[i] < rhs.vertices[i];
    }
    return false;
}

} // namespace CDT

namespace
{

template <typename T, typename TNearPointLocator>
EdgeUSet extractAllEdges(const CDT::Triangulation<T, TNearPointLocator>& cdt)
{
    EdgeUSet out;
    for(const auto& t : cdt.triangles)
    {
        for(std::size_t i = 0; i < 3; ++i)
        {
            out.insert(Edge(t.vertices[i], t.vertices[(i + 1) % 3]));
        }
    }
    return out;
}

std::ostream& operator<<(std::ostream& o, const Edge& e)
{
    o << e.v1() << " " << e.v2() << '\n';
    return o;
}

std::ostream& operator<<(std::ostream& o, const std::vector<Edge>& edges)
{
    for(const auto& e : edges)
        o << e;
    return o;
}

std::vector<Edge> sortedEdges(const EdgeUSet& edges)
{
    std::vector<Edge> out(edges.begin(), edges.end());
    std::sort(out.begin(), out.end());
    return out;
}

template <typename T>
std::pair<Vertices<T>, std::vector<Edge> >
readInputFromFile(const std::string& fileName)
{
    std::ifstream fin(fileName);
    if(!fin.is_open())
    {
        ENHANCED_THROW(std::runtime_error, "Could not open file");
    }
    IndexSizeType nVerts;
    IndexSizeType nEdges;
    fin >> nVerts >> nEdges;

    // Read vertices
    Vertices<T> vv;
    vv.reserve(nVerts);
    for(std::size_t i = 0; i < nVerts; ++i)
    {
        T x, y;
        fin >> x >> y;
        vv.push_back(V2d<T>::make(x, y));
    }
    // Read edges
    std::vector<Edge> ee;
    for(std::size_t i = 0; i < nEdges; ++i)
    {
        VertInd v1, v2;
        fin >> v1 >> v2;
        ee.emplace_back(v1, v2);
    }
    return std::make_pair(vv, ee);
}

template <typename T, typename TNearPointLocator>
void inputFromFile(
    CDT::Triangulation<T, TNearPointLocator>& cdt,
    const std::string& filename)
{
    const auto [vv, ee] = readInputFromFile<T>(filename);
    cdt.insertVertices(vv);
    cdt.insertEdges(ee);
}

template <typename T, typename TNearPointLocator>
void saveToOff(
    const std::string& filename,
    const CDT::Triangulation<T, TNearPointLocator>& cdt)
{
    std::ofstream fout(filename);
    if(!fout.is_open())
    {
        ENHANCED_THROW(
            std::runtime_error, "Save can't open file for writing OFF");
    }
    fout.precision(std::numeric_limits<T>::digits10 + 1);
    fout << "OFF\n";
    fout << cdt.vertices.size() << ' ' << cdt.triangles.size() << " 0\n";
    // Write vertices
    for(const auto& v : cdt.vertices)
    {
        fout << v.x << ' ' << v.y << ' ' << 0 << "\n";
    }
    // Write faces
    for(const auto& t : cdt.triangles)
    {
        fout << "3 " << t.vertices[0] << ' ' << t.vertices[1] << ' '
             << t.vertices[2] << "\n";
    }
    fout.close();
}

} // namespace

TEMPLATE_LIST_TEST_CASE("Triangulation Tests", "", CoordTypes)
{
    auto cdt = Triangulation<TestType>();
    cdt.insertVertices(Vertices<TestType>{{0, 0}, {1, 1}, {3, 1}, {3, 0}});
    REQUIRE(CDT::verifyTopology(cdt));
    REQUIRE(cdt.vertices.size() == std::size_t(4 + 3));
    REQUIRE(cdt.fixedEdges.size() == std::size_t(0));
    REQUIRE(cdt.triangles.size() == std::size_t(9));
    SECTION("Edges have indices offsetted by 3 because of super-triangle")
    {
        const auto edges = extractAllEdges(cdt);
        REQUIRE(edges.count(Edge(1 + 3, 3 + 3)));
    }
    SECTION(
        "Erasing super-triangle affects indices in edges and triangle count")
    {
        cdt.eraseSuperTriangle();
        REQUIRE(CDT::verifyTopology(cdt));
        REQUIRE(cdt.triangles.size() == std::size_t(2));
        const auto edges = extractAllEdges(cdt);
        REQUIRE(edges.count(Edge(1, 3)));
    }
    SECTION("Adding a constraint edge")
    {
        const auto constraintEdge = Edge(0, 2);
        cdt.insertEdges(std::vector<Edge>{constraintEdge});
        cdt.eraseSuperTriangle();
        REQUIRE(CDT::verifyTopology(cdt));
        REQUIRE(cdt.vertices.size() == std::size_t(4));
        REQUIRE(cdt.fixedEdges.size() == std::size_t(1));
        REQUIRE(cdt.triangles.size() == std::size_t(2));
        REQUIRE(cdt.fixedEdges.count(constraintEdge));
        const auto edges = extractAllEdges(cdt);
        REQUIRE(edges.count(constraintEdge));
    }
}

namespace
{

Triangle makeSmallestIndexFirst(const Triangle& t)
{
    const auto& vv = t.vertices;
    const Index iStart =
        vv[0] < vv[1] ? (vv[0] < vv[2] ? 0 : 2) : (vv[1] < vv[2] ? 1 : 2);
    Triangle out;
    for(Index i = 0; i < Index(3); ++i)
    {
        const Index j = (iStart + i) % 3;
        out.vertices[i] = t.vertices[j];
        out.neighbors[i] = t.neighbors[j];
    }
    return out;
}

// return mapping of index in a not sorted array to index in sorted array
template <typename RandomIt>
auto sort_mapping(RandomIt cbegin, RandomIt cend)
{
    auto len = std::distance(cbegin, cend);
    std::vector<size_t> perm(len);
    std::iota(perm.begin(), perm.end(), 0U);
    std::sort(perm.begin(), perm.end(), [&](const size_t& a, const size_t& b) {
        return *(cbegin + a) < *(cbegin + b);
    });
    std::vector<size_t> map(len);
    for(size_t i = 0; i < perm.size(); ++i)
        map[perm[i]] = i;
    return map;
}

/// Triangulation topology stored in a sorted way that makes it easy to perform
/// baseline comparisons
struct TriangulationTopo
{
    TriangleVec triangles;
    EdgeVec fixedEdges;
    std::map<Edge, BoundaryOverlapCount> overlapCount;
    std::map<Edge, EdgeVec> pieceToOriginals;

    TriangulationTopo() = default;

    template <typename T, typename TNearPointLocator>
    TriangulationTopo(const Triangulation<T, TNearPointLocator>& cdt)
        : triangles(cdt.triangles)
        , fixedEdges(cdt.fixedEdges.begin(), cdt.fixedEdges.end())
        , overlapCount(cdt.overlapCount.begin(), cdt.overlapCount.end())
        , pieceToOriginals(
              cdt.pieceToOriginals.begin(),
              cdt.pieceToOriginals.end())
    {
        for(auto& t : triangles)
        {
            t = makeSmallestIndexFirst(t);
        }
        std::sort(fixedEdges.begin(), fixedEdges.end());
        const auto sortMapping =
            sort_mapping(triangles.begin(), triangles.end());
        std::sort(triangles.begin(), triangles.end());
        // remap neighbor indices according to new sorted order
        for(auto& t : triangles)
        {
            for(auto& n : t.neighbors)
            {
                if(n != noNeighbor)
                {
                    n = TriInd(sortMapping[n]);
                }
            }
        }
    }

    void read(std::istream& f)
    {
        if(!f.good())
        {
            ENHANCED_THROW(std::runtime_error, "Could not open file");
        }
        std::size_t size;

        triangles = TriangleVec();
        f >> size;
        triangles.resize(size);
        for(auto& t : triangles)
        {
            for(auto& v : t.vertices)
                f >> v;
            for(auto& n : t.neighbors)
                f >> n;
        }

        fixedEdges = EdgeVec();
        f >> size;
        fixedEdges.reserve(size);
        VertInd v1, v2;
        for(IndexSizeType i = 0; i < size; ++i)
        {
            f >> v1 >> v2;
            fixedEdges.emplace_back(v1, v2);
        }

        overlapCount = std::map<Edge, BoundaryOverlapCount>();
        BoundaryOverlapCount boc;
        f >> size;
        for(IndexSizeType i = 0; i < size; ++i)
        {
            f >> v1 >> v2 >> boc;
            overlapCount[Edge(v1, v2)] = boc;
        }

        pieceToOriginals = std::map<Edge, EdgeVec>();
        f >> size;
        std::size_t edgeVecSize;
        for(IndexSizeType i = 0; i < size; ++i)
        {
            f >> v1 >> v2 >> edgeVecSize;
            const auto e = Edge(v1, v2);
            auto edges = EdgeVec();
            edges.reserve(edgeVecSize);
            for(IndexSizeType j = 0; j < edgeVecSize; ++j)
            {
                f >> v1 >> v2;
                edges.emplace_back(v1, v2);
            }
            pieceToOriginals[e] = edges;
        }
    }

    void write(std::ostream& f) const
    {
        if(!f.good())
        {
            ENHANCED_THROW(std::runtime_error, "Could not open file");
        }
        f << triangles.size() << '\n';
        for(const auto& t : triangles)
        {
            const auto& vv = t.vertices;
            const auto& nn = t.neighbors;
            f << vv[0] << ' ' << vv[1] << ' ' << vv[2] << "   " << nn[0] << ' '
              << nn[1] << ' ' << nn[2] << '\n';
        }
        f << '\n' << fixedEdges.size() << '\n';
        for(const auto& e : fixedEdges)
        {
            f << e.v1() << ' ' << e.v2() << '\n';
        }
        f << '\n' << overlapCount.size() << '\n';
        for(const auto& oc : overlapCount)
        {
            f << oc.first.v1() << ' ' << oc.first.v2() << "    " << oc.second
              << '\n';
        }
        f << '\n' << pieceToOriginals.size() << '\n';
        for(const auto& pto : pieceToOriginals)
        {
            f << pto.first.v1() << ' ' << pto.first.v2() << '\n'
              << "    " << pto.second.size() << '\n';
            for(const auto& e : pto.second)
            {
                f << "    " << e.v1() << ' ' << e.v2() << '\n';
            }
        }
    }
    void writeToFile(const std::string& filename) const
    {
        std::ofstream f(filename);
        if(!f.is_open())
        {
            ENHANCED_THROW(std::runtime_error, "Can't open file for writing");
        }
        write(f);
    }
};

template <typename T, typename TNearPointLocator>
std::string topologyString(const CDT::Triangulation<T, TNearPointLocator>& cdt)
{
    std::ostringstream out;
    TriangulationTopo(cdt).write(out);
    return out.str();
}

std::string topologyString(const std::string& filename)
{
    std::ifstream ifs(filename);
    return std::string(std::istreambuf_iterator<char>(ifs), {});
}

} // namespace

TEMPLATE_LIST_TEST_CASE(
    "Test triangulation topology read/write",
    "",
    CoordTypes)
{
    const auto [vv, ee] =
        readInputFromFile<TestType>("inputs/corner cases.txt");
    auto cdt = Triangulation<TestType>(
        CDT::VertexInsertionOrder::Randomized,
        CDT::IntersectingConstraintEdges::Resolve,
        0.);
    cdt.insertVertices(vv);
    cdt.insertEdges(ee);
    cdt.eraseSuperTriangle();
    const auto topo = TriangulationTopo(cdt);

    std::ostringstream out_s;
    std::ofstream f("tmp.txt");
    if(!f.is_open())
    {
        ENHANCED_THROW(std::runtime_error, "Can't open file for writing");
    }
    topo.write(f);
    topo.write(out_s);
    std::istringstream in_s(out_s.str());
    TriangulationTopo topo2;
    topo2.read(in_s);
    std::ostringstream out_s2;
    topo2.write(out_s2);
    REQUIRE(out_s.str() == out_s2.str());
}

TEMPLATE_LIST_TEST_CASE("Test ground truth tests", "", CoordTypes)
{
    const auto data = GENERATE(table<
                               std::string,
                               std::string,
                               VertexInsertionOrder::Enum,
                               IntersectingConstraintEdges::Enum,
                               TestType>(
        // clang-format off
{
        {"corner cases.txt", "corner cases.txt", VertexInsertionOrder::Randomized, IntersectingConstraintEdges::Resolve, TestType(0.)},
        {"corner cases.txt", "corner cases.txt", VertexInsertionOrder::AsProvided, IntersectingConstraintEdges::Resolve, TestType(0.)}
        }));
    // clang-format on

    const auto& inputFile = std::get<0>(data);
    const auto& expectedOutputFile = std::get<1>(data);
    const auto order = std::get<2>(data);
    const auto intersectingEdgesStrategy = std::get<3>(data);
    const auto minDistToConstraintEdge = std::get<4>(data);

    auto cdt = Triangulation<TestType>(
        order, intersectingEdgesStrategy, minDistToConstraintEdge);
    inputFromFile(cdt, "inputs/" + inputFile);
    cdt.eraseSuperTriangle();
    TriangulationTopo(cdt).writeToFile("tmp.txt");
    REQUIRE(
        topologyString(cdt) ==
        topologyString("expected/" + expectedOutputFile));
}