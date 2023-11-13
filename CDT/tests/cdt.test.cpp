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
#include <map>
#include <numeric>
#include <sstream>

using namespace CDT;

constexpr bool updateFiles = false;

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
    for(Index i(0); i < Index(3); ++i)
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

[[maybe_unused]] std::ostream&
operator<<(std::ostream& o, const std::vector<Edge>& edges)
{
    for(const auto& e : edges)
        o << e;
    return o;
}

[[maybe_unused]] std::vector<Edge> sortedEdges(const EdgeUSet& edges)
{
    std::vector<Edge> out(edges.begin(), edges.end());
    std::sort(out.begin(), out.end());
    return out;
}

template <typename T>
std::pair<Vertices<T>, std::vector<Edge> >
readInputFromFile(const std::string& fileName)
{
    std::ifstream f(fileName);
    if(!f.is_open())
    {
        ENHANCED_THROW(
            std::runtime_error, "Could not open file '" + fileName + '\'');
    }
    f.precision(std::numeric_limits<T>::digits10 + 1);
    IndexSizeType nVerts;
    IndexSizeType nEdges;
    f >> nVerts >> nEdges;

    // Read vertices
    Vertices<T> vv;
    vv.reserve(nVerts);
    for(std::size_t i = 0; i < nVerts; ++i)
    {
        T x, y;
        f >> x >> y;
        vv.push_back(V2d<T>::make(x, y));
    }
    // Read edges
    std::vector<Edge> ee;
    for(std::size_t i = 0; i < nEdges; ++i)
    {
        VertInd v1, v2;
        f >> v1 >> v2;
        ee.emplace_back(v1, v2);
    }
    return std::make_pair(vv, ee);
}

template <typename T>
void saveInputToFile(
    const std::string& fileName,
    const Vertices<T>& vv,
    const std::vector<Edge>& ee)
{
    std::ofstream f(fileName);
    if(!f.is_open())
    {
        ENHANCED_THROW(std::runtime_error, "Could not open file");
    }
    IndexSizeType nVerts;
    IndexSizeType nEdges;
    f << vv.size() << ' ' << ee.size() << '\n';
    for(const auto& v : vv)
    {
        f << v.x << ' ' << v.y << '\n';
    }
    for(const auto& e : ee)
    {
        f << e.v1() << ' ' << e.v2() << '\n';
    }
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
        REQUIRE(edges.count(Edge(VertInd(1 + 3), VertInd(3 + 3))));
    }
    SECTION(
        "Erasing super-triangle affects indices in edges and triangle count")
    {
        cdt.eraseSuperTriangle();
        REQUIRE(CDT::verifyTopology(cdt));
        REQUIRE(cdt.triangles.size() == std::size_t(2));
        const auto edges = extractAllEdges(cdt);
        REQUIRE(edges.count(Edge(VertInd(1), VertInd(3))));
    }
    SECTION("Adding a constraint edge")
    {
        const auto constraintEdge = Edge(VertInd(0), VertInd(2));
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
    const Index iStart = vv[0] < vv[1] ? Index(vv[0] < vv[2] ? 0 : 2)
                                       : Index(vv[1] < vv[2] ? 1 : 2);
    Triangle out;
    for(Index i(0); i < Index(3); ++i)
    {
        const auto j = Index((iStart + i) % 3);
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

template <typename T, typename TNearPointLocator>
void topologyToFile(
    const std::string& filename,
    const CDT::Triangulation<T, TNearPointLocator>& cdt)
{
    std::ofstream out(filename);
    TriangulationTopo(cdt).write(out);
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
        CDT::VertexInsertionOrder::Auto,
        CDT::IntersectingConstraintEdges::TryResolve,
        0.);
    cdt.insertVertices(vv);
    cdt.insertEdges(ee);
    cdt.eraseSuperTriangle();
    const auto topo = TriangulationTopo(cdt);
    std::ostringstream out_s;
    topo.write(out_s);
    std::istringstream in_s(out_s.str());
    TriangulationTopo topo2;
    topo2.read(in_s);
    std::ostringstream out_s2;
    topo2.write(out_s2);
    REQUIRE(out_s.str() == out_s2.str());
}

namespace
{

std::string to_string(const VertexInsertionOrder::Enum& vio)
{
    switch(vio)
    {
    case VertexInsertionOrder::AsProvided:
        return "as-provided";
    case VertexInsertionOrder::Auto:
        return "auto";
    }
    ENHANCED_THROW(std::runtime_error, "Reached unreachable");
}

std::string to_string(const IntersectingConstraintEdges::Enum& ice)
{
    switch(ice)
    {
    case IntersectingConstraintEdges::DontCheck:
        return "no-check";
    case IntersectingConstraintEdges::NotAllowed:
        return "ignore";
    case IntersectingConstraintEdges::TryResolve:
        return "resolve";
    }
    ENHANCED_THROW(std::runtime_error, "Reached unreachable");
}

} // namespace

TEMPLATE_LIST_TEST_CASE(
    "Ground truth tests: constraint triangulation",
    "",
    CoordTypes)
{
    const auto inputFile = GENERATE(
        as<std::string>{},
        "Capital A.txt",
        "cdt.txt",
        "ditch.txt",
        "double-hanging.txt",
        "gh_issue.txt",
        "guitar no box.txt",
        "Hanging.txt",
        "Hanging2.txt",
        "island.txt",
        "issue-142-double-hanging-edge.txt",
        "issue-42-full-boundary-overlap.txt",
        "issue-42-hole-overlaps-bondary.txt",
        "issue-42-multiple-boundary-overlaps-conform-to-edge.txt",
        "issue-42-multiple-boundary-overlaps.txt",
        "issue-65-wrong-edges.txt",
        "kidney.txt",
        "Letter u.txt",
        "OnEdge.txt",
        "overlapping constraints.txt",
        "overlapping constraints2.txt",
        "points_on_constraint_edge.txt",
        "ProblematicCase1.txt",
        "regression_issue_38_wrong_hull_small.txt",
        "square with crack.txt",
        "test_data_small.txt",
        "triple-hanging-flipped.txt",
        "triple-hanging.txt",
        "unit square.txt");

    const auto typeSpecific = std::unordered_set<std::string>{
        "guitar no box.txt",
        "issue-42-full-boundary-overlap.txt",
        "issue-42-multiple-boundary-overlaps-conform-to-edge.txt",
        "issue-42-multiple-boundary-overlaps.txt",
        "overlapping constraints.txt"};

    const std::string typeString =
        typeSpecific.count(inputFile)
            ? sizeof(TestType) == sizeof(double) ? "f64_" : "f32_"
            : "";

    const auto order =
        GENERATE(VertexInsertionOrder::AsProvided, VertexInsertionOrder::Auto);
    const auto intersectingEdgesStrategy = GENERATE(
        IntersectingConstraintEdges::NotAllowed,
        IntersectingConstraintEdges::TryResolve);
    const auto minDistToConstraintEdge = 0.;

    auto cdt = Triangulation<TestType>(
        order, intersectingEdgesStrategy, minDistToConstraintEdge);
    const auto [vv, ee] = readInputFromFile<TestType>("inputs/" + inputFile);
    const DuplicatesInfo di = FindDuplicates<TestType>(
        vv.begin(), vv.end(), getX_V2d<TestType>, getY_V2d<TestType>);
    INFO("Input file is '" + inputFile + "'");
    REQUIRE(di.duplicates.empty());
    cdt.insertVertices(vv);
    cdt.insertEdges(ee);
    REQUIRE(CDT::verifyTopology(cdt));
    REQUIRE(CDT::eachVertexHasNeighborTriangle(cdt));

    // make true to update expected files (development purposes only)
    const auto outputFileBase = "expected/" +
                                inputFile.substr(0, inputFile.size() - 4) +
                                "__" + typeString + to_string(order) + "_" +
                                to_string(intersectingEdgesStrategy);
    SECTION("With super-triangle")
    {
        const auto outFile = outputFileBase + "_all.txt";
        INFO("Output file is '" + outFile + "'");
        if(updateFiles)
            topologyToFile(outFile, cdt);
        else
        {
            REQUIRE(topologyString(cdt) == topologyString(outFile));
        }
    }
    SECTION("Erase super-triangle")
    {
        const auto outFile = outputFileBase + "_super.txt";
        INFO("Output file is '" + outFile + "'");
        cdt.eraseSuperTriangle();
        if(updateFiles)
            topologyToFile(outFile, cdt);
        else
            REQUIRE(topologyString(cdt) == topologyString(outFile));
    }
    SECTION("Erase outer triangles")
    {
        const auto outFile = outputFileBase + "_outer.txt";
        INFO("Output file is '" + outFile + "'");
        cdt.eraseOuterTriangles();
        if(updateFiles)
            topologyToFile(outFile, cdt);
        else
            REQUIRE(topologyString(cdt) == topologyString(outFile));
    }
    SECTION("Auto detect holes and erase outer triangles")
    {
        const auto outFile = outputFileBase + "_auto.txt";
        INFO("Output file is '" + outFile + "'");
        cdt.eraseOuterTrianglesAndHoles();
        if(updateFiles)
            topologyToFile(outFile, cdt);
        else
            REQUIRE(topologyString(cdt) == topologyString(outFile));
    }
}

TEMPLATE_LIST_TEST_CASE(
    "Ground truth tests: conforming triangulation",
    "",
    CoordTypes)
{
    const auto inputFile = GENERATE(
        as<std::string>{},
        "Capital A.txt",
        "cdt.txt",
        "ditch.txt",
        "double-hanging.txt",
        "gh_issue.txt",
        "guitar no box.txt",
        "Hanging2.txt",
        "issue-142-double-hanging-edge.txt",
        "issue-42-multiple-boundary-overlaps.txt",
        "points_on_constraint_edge.txt",
        "ProblematicCase1.txt",
        "triple-hanging-flipped.txt",
        "triple-hanging.txt",
        "unit square.txt");

    const auto typeSpecific = std::unordered_set<std::string>{
        "guitar no box.txt",
        "issue-42-multiple-boundary-overlaps.txt",
    };

    const std::string typeString =
        typeSpecific.count(inputFile)
            ? sizeof(TestType) == sizeof(double) ? "f64_" : "f32_"
            : "";

    const auto order = VertexInsertionOrder::Auto;
    const auto intersectingEdgesStrategy =
        IntersectingConstraintEdges::NotAllowed;
    const auto minDistToConstraintEdge = 0.;

    auto cdt = Triangulation<TestType>(
        order, intersectingEdgesStrategy, minDistToConstraintEdge);
    const auto [vv, ee] = readInputFromFile<TestType>("inputs/" + inputFile);
    const DuplicatesInfo di = FindDuplicates<TestType>(
        vv.begin(), vv.end(), getX_V2d<TestType>, getY_V2d<TestType>);
    INFO("Input file is '" + inputFile + "'");
    REQUIRE(di.duplicates.empty());
    cdt.insertVertices(vv);
    cdt.conformToEdges(ee);
    REQUIRE(CDT::verifyTopology(cdt));

    // make true to update expected files (development purposes only)
    const auto outputFileBase =
        "expected/" + inputFile.substr(0, inputFile.size() - 4) +
        "__conforming_" + typeString + to_string(order) + "_" +
        to_string(intersectingEdgesStrategy);
    {
        const auto outFile = outputFileBase + "_auto.txt";
        INFO("Output file is '" + outFile + "'");
        cdt.eraseOuterTrianglesAndHoles();
        if(updateFiles)
            topologyToFile(outFile, cdt);
        else
            REQUIRE(topologyString(cdt) == topologyString(outFile));
    }
}

TEMPLATE_LIST_TEST_CASE("Ground truth tests: crossing edges", "", CoordTypes)
{
    const auto inputFile = GENERATE(
        as<std::string>{},
        "crossing-edges.txt",
        "issue-148-crossing-edges.txt");

    const auto order = VertexInsertionOrder::Auto;
    const auto intersectingEdgesStrategy =
        IntersectingConstraintEdges::TryResolve;
    const auto minDistToConstraintEdge = 0.;

    auto cdt = Triangulation<TestType>(
        order, intersectingEdgesStrategy, minDistToConstraintEdge);
    const auto [vv, ee] = readInputFromFile<TestType>("inputs/" + inputFile);
    const DuplicatesInfo di = FindDuplicates<TestType>(
        vv.begin(), vv.end(), getX_V2d<TestType>, getY_V2d<TestType>);
    INFO("Input file is '" + inputFile + "'");
    REQUIRE(di.duplicates.empty());
    const auto triangulationType =
        GENERATE(as<std::string>{}, "", "conforming_");
    cdt.insertVertices(vv);
    triangulationType == "" ? cdt.insertEdges(ee) : cdt.conformToEdges(ee);
    REQUIRE(CDT::verifyTopology(cdt));
    REQUIRE(CDT::eachVertexHasNeighborTriangle(cdt));
    // make true to update expected files (development purposes only)
    const auto outputFileBase = "expected/" +
                                inputFile.substr(0, inputFile.size() - 4) +
                                "__" + triangulationType + to_string(order) +
                                "_" + to_string(intersectingEdgesStrategy);
    {
        const auto outFile = outputFileBase + "_all.txt";
        INFO("Output file is '" + outFile + "'");
        if(updateFiles)
            topologyToFile(outFile, cdt);
        else
            REQUIRE(topologyString(cdt) == topologyString(outFile));
    }
}

TEMPLATE_LIST_TEST_CASE("Inserting vertices in two batches", "", CoordTypes)
{
    const auto [vv, ee] =
        readInputFromFile<TestType>("inputs/Constrained Sweden.txt");
    auto cdt = Triangulation<TestType>(
        CDT::VertexInsertionOrder::Auto,
        CDT::IntersectingConstraintEdges::TryResolve,
        0.);
    cdt.insertVertices(vv);
    cdt.insertEdges(ee);
    cdt.eraseOuterTrianglesAndHoles();

    const auto halfSize = vv.size() / 2;
    const auto vv_lo = Vertices<TestType>(vv.begin(), vv.begin() + halfSize);
    const auto vv_hi = Vertices<TestType>(vv.begin() + halfSize, vv.end());
    auto cdtBatches = Triangulation<TestType>(
        CDT::VertexInsertionOrder::Auto,
        CDT::IntersectingConstraintEdges::TryResolve,
        0.);
    cdtBatches.insertVertices(vv_lo);
    cdtBatches.insertVertices(vv_hi);
    cdtBatches.insertEdges(ee);
    cdtBatches.eraseOuterTrianglesAndHoles();

    REQUIRE(topologyString(cdt) == topologyString(cdtBatches));
}

TEMPLATE_LIST_TEST_CASE("Benchmarks", "[benchmark][.]", CoordTypes)
{
    SECTION("Constrained Sweden")
    {
        Vertices<TestType> vv;
        std::vector<Edge> ee;
        tie(vv, ee) =
            readInputFromFile<TestType>("inputs/Constrained Sweden.txt");
        BENCHMARK("Constrained Sweden (vertices only): As Provided")
        {
            auto cdt =
                Triangulation<TestType>(VertexInsertionOrder::AsProvided);
            cdt.insertVertices(vv);
        };
        BENCHMARK("Constrained Sweden (vertices only): Auto")
        {
            auto cdt = Triangulation<TestType>(VertexInsertionOrder::Auto);
            cdt.insertVertices(vv);
        };
        BENCHMARK("Constrained Sweden: As Provided")
        {
            auto cdt =
                Triangulation<TestType>(VertexInsertionOrder::AsProvided);
            cdt.insertVertices(vv);
            cdt.insertEdges(ee);
        };
        BENCHMARK("Constrained Sweden: Auto")
        {
            auto cdt = Triangulation<TestType>(VertexInsertionOrder::Auto);
            cdt.insertVertices(vv);
            cdt.insertEdges(ee);
        };
    }
}

TEST_CASE("Don't flip constraint edge when resolving intersection", "")
{
    const auto inputFile =
        std::string("dont_flip_constraint_when_resolving_intersection.txt");
    const auto order = VertexInsertionOrder::AsProvided;
    const auto intersectingEdgesStrategy =
        IntersectingConstraintEdges::TryResolve;
    const auto minDistToConstraintEdge = 1e-6;
    const auto outFile = "expected/" +
                         inputFile.substr(0, inputFile.size() - 4) + "__f64_" +
                         to_string(order) + "_" +
                         to_string(intersectingEdgesStrategy) + "_all.txt";

    const auto [vv, ee] = readInputFromFile<double>("inputs/" + inputFile);
    auto cdt = Triangulation<double>(
        order, intersectingEdgesStrategy, minDistToConstraintEdge);
    cdt.insertVertices(vv);
    cdt.insertEdges(ee);
    REQUIRE(CDT::verifyTopology(cdt));

    if(updateFiles)
        topologyToFile(outFile, cdt);
    else
    {
        REQUIRE(topologyString(cdt) == topologyString(outFile));
    }
}

TEST_CASE(
    "Regression: resolving edges intersection with a hanging edge in a "
    "pseudo-polygon",
    "")
{
    const auto inputFile = std::string("HangingIntersection.txt");
    const auto order = VertexInsertionOrder::Auto;
    const auto intersectingEdgesStrategy =
        IntersectingConstraintEdges::TryResolve;
    const auto minDistToConstraintEdge = 1e-6;
    const auto outFile = "expected/" +
                         inputFile.substr(0, inputFile.size() - 4) + "__f64_" +
                         to_string(order) + "_" +
                         to_string(intersectingEdgesStrategy) + "_all.txt";

    const auto [vv, ee] = readInputFromFile<double>("inputs/" + inputFile);
    auto cdt = Triangulation<double>(
        order, intersectingEdgesStrategy, minDistToConstraintEdge);
    cdt.insertVertices(vv);
    cdt.insertEdges(ee);
    REQUIRE(CDT::verifyTopology(cdt));

    if(updateFiles)
        topologyToFile(outFile, cdt);
    else
    {
        REQUIRE(topologyString(cdt) == topologyString(outFile));
    }
}

TEST_CASE("Regression: multiple hanging edges", "")
{
    const auto inputFile = std::string("HangingIntersection.txt");
    const auto order = VertexInsertionOrder::Auto;
    const auto intersectingEdgesStrategy =
        IntersectingConstraintEdges::TryResolve;
    const auto minDistToConstraintEdge = 1e-6;
    const auto outFile = "expected/" +
                         inputFile.substr(0, inputFile.size() - 4) + "__f64_" +
                         to_string(order) + "_" +
                         to_string(intersectingEdgesStrategy) + "_all.txt";

    const auto [vv, ee] = readInputFromFile<double>("inputs/" + inputFile);
    auto cdt = Triangulation<double>(
        order, intersectingEdgesStrategy, minDistToConstraintEdge);
    cdt.insertVertices(vv);
    cdt.insertEdges(ee);
    REQUIRE(CDT::verifyTopology(cdt));

    if(updateFiles)
        topologyToFile(outFile, cdt);
    else
    {
        REQUIRE(topologyString(cdt) == topologyString(outFile));
    }
}

TEST_CASE("Regression test", "")
{
    const auto inputFile = std::string("debug2.txt");
    const auto [vv, ee] = readInputFromFile<double>("inputs/" + inputFile);
    auto cdt = Triangulation<double>(VertexInsertionOrder::Auto);
    cdt.insertVertices(vv);
    cdt.insertEdges(ee);
    REQUIRE(CDT::verifyTopology(cdt));
}

TEST_CASE("Regression test issue #154 (1)", "")
{
    // Very large coordinate values lead to wrong super-triangle coordinates due
    // to the floating-point rounding
    auto cdt = Triangulation<double>{};
    cdt.insertVertices({
        {0.0, 1e38},
        {1.0, 1e38},
    });
    REQUIRE(CDT::verifyTopology(cdt));
    const auto outFile = "expected/154_1.txt";
    if(updateFiles)
        topologyToFile(outFile, cdt);
    else
    {
        REQUIRE(topologyString(cdt) == topologyString(outFile));
    }
}

TEST_CASE("Regression test issue #154 (2)", "")
{
    // Explanation: There was an incorrect assumptions that there are no 'loops'
    // in the pseudo-polygons, only 'hanging' edges. The loops are possible as
    // shown by this test case.
    auto cdt = Triangulation<double>{};
    cdt.insertVertices({
        {2.0, -2.18933983E-5},
        {-0.0896810815, -2.18407786E-5},
        {-2.19008489E-5, -7.64692231E-6},
        {8.73939061E-5, 0.00568488613},
        {-0.00142463227, -0.00142461748},
        {-7.67273832E-6, 8.7602064E-5},
        {0.00569847599, -0.00142463227},
        {-2.18156383E-5, -7.6295637E-6},
    });
    REQUIRE(CDT::verifyTopology(cdt));
    cdt.insertEdges({
        {0, 1},
    });
    REQUIRE(CDT::verifyTopology(cdt));
    const auto outFile = "expected/154_2.txt";
    if(updateFiles)
        topologyToFile(outFile, cdt);
    else
    {
        REQUIRE(topologyString(cdt) == topologyString(outFile));
    }
}

TEST_CASE("Regression test issue #154 (3)", "")
{
    // Explanation: There was an incorrect assumptions that there are no 'loops'
    // in the pseudo-polygons, only 'hanging' edges. The loops are possible as
    // shown by this test case.
    auto cdt = Triangulation<double>{};
    cdt.insertVertices({
        {-2.0, 1.47656155},
        {-6.40527344, -40.4999084},
        {0.0, -7.96960115},
        {-2.00152564, 1.46877956},
        {-2.70361328, -7.99999619},
        {-2.70465064, -7.99901962},
        {-7.97778273, -19.3754253},
        {7.96885204, -5.37488127},
        {-7.97180128, -39.7499695},
    });
    cdt.insertEdges({
        {0, 8},
    });
    REQUIRE(CDT::verifyTopology(cdt));
    const auto outFile = "expected/154_3.txt";
    if(updateFiles)
        topologyToFile(outFile, cdt);
    else
    {
        REQUIRE(topologyString(cdt) == topologyString(outFile));
    }
}

TEST_CASE("Regression test: hanging edge in pseudo-poly", "")
{
    auto [vv, ee] = readInputFromFile<double>("inputs/hanging3.txt");
    auto cdt = Triangulation<double>{};
    cdt.insertVertices(vv);
    cdt.insertEdges(ee);
    REQUIRE(CDT::verifyTopology(cdt));
}
