#include <CDT.h>

#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace CDT;

using CoordTypes = std::tuple<float, double>;
template <class CoordType>
using Vertices = std::vector<V2d<CoordType> >;

namespace
{

template <typename T, typename TNearPointLocator>
EdgeUSet ExtractAllEdges(const CDT::Triangulation<T, TNearPointLocator>& cdt)
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

} // namespace

TEMPLATE_LIST_TEST_CASE("Triangulation Tests", "", CoordTypes)
{
    auto cdt = Triangulation<TestType>();
    cdt.insertVertices(Vertices<TestType>{{0, 0}, {1, 1}, {3, 1}, {3, 0}});
    REQUIRE(cdt.vertices.size() == std::size_t(4 + 3));
    REQUIRE(cdt.fixedEdges.size() == std::size_t(0));
    REQUIRE(cdt.triangles.size() == std::size_t(9));
    SECTION("Edges have indices offsetted by 3 because of super-triangle")
    {
        const auto edges = ExtractAllEdges(cdt);
        REQUIRE(edges.count(Edge(1 + 3, 3 + 3)));
    }
    SECTION(
        "Erasing super-triangle affects indices in edges and triangle count")
    {
        cdt.eraseSuperTriangle();
        REQUIRE(cdt.triangles.size() == std::size_t(2));
        const auto edges = ExtractAllEdges(cdt);
        REQUIRE(edges.count(Edge(1, 3)));
    }
    SECTION("Adding a constraint edge")
    {
        const auto constraintEdge = Edge(0, 2);
        cdt.insertEdges(std::vector<Edge>{constraintEdge});
        cdt.eraseSuperTriangle();
        REQUIRE(cdt.vertices.size() == std::size_t(4));
        REQUIRE(cdt.fixedEdges.size() == std::size_t(1));
        REQUIRE(cdt.triangles.size() == std::size_t(2));
        REQUIRE(cdt.fixedEdges.count(constraintEdge));
        const auto edges = ExtractAllEdges(cdt);
        REQUIRE(edges.count(constraintEdge));
    }
}
