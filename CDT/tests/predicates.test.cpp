/* Tests for the robust predicates in `include/predicates.h`. Test material is
 * borrowed from the public test-suites of other implementations of Shewchuk's
 * predicates; individual test cases name their source:
 *  - github.com/mourner/robust-predicates (Vladimir Agafonkin, Unlicense)
 *  - github.com/georust/robust (Spade/GeoRust Developers, MIT OR Apache-2.0)
 *  - www.cs.cmu.edu/~quake/robust.html, `predicates.c` (Shewchuk, public
 *    domain)
 * The hard-case fixtures have their own provenance and license notes in
 * `inputs/predicates/README.md`.
 */

#include <predicates.h>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#define REQUIRE_NO_FAILURES(log)                                               \
    INFO("failed cases: " << (log).count << ", first one: " << (log).first);   \
    REQUIRE((log).count == 0)

namespace
{

using CoordTypes = std::tuple<float, double>;

template <typename T>
int signOf(const T x)
{
    return x > T(0) ? 1 : (x < T(0) ? -1 : 0);
}

/// Keeps Catch2 assertions out of hot loops: hundreds of thousands of
/// REQUIREs are slow, and unreadable when they fail
struct FailureLog
{
    std::size_t count;
    std::string first;

    FailureLog()
        : count(0)
    {}

    void add(const std::string& what)
    {
        if(count++ == 0)
            first = what;
    }
};

/* Exact integer oracle: the predicates are polynomials in the coordinates, so
 * for small integer coordinates the same determinants are exact in `int64_t`.
 * Largest coordinate that cannot overflow an intermediate: 2^20 for orient2d
 * (degree 2), 2^12 for incircle (4), 2^17 for orient3d (3), 2^8 for insphere
 * (5). */

using ExactInt = std::int64_t;

ExactInt
det2(const ExactInt a, const ExactInt b, const ExactInt c, const ExactInt d)
{
    return a * d - b * c;
}

ExactInt det3(
    const ExactInt a,
    const ExactInt b,
    const ExactInt c,
    const ExactInt d,
    const ExactInt e,
    const ExactInt f,
    const ExactInt g,
    const ExactInt h,
    const ExactInt i)
{
    return a * det2(e, f, h, i) - b * det2(d, f, g, i) + c * det2(d, e, g, h);
}

ExactInt orient2dInt(const ExactInt* a, const ExactInt* b, const ExactInt* c)
{
    return det2(a[0] - c[0], a[1] - c[1], b[0] - c[0], b[1] - c[1]);
}

ExactInt incircleInt(
    const ExactInt* a,
    const ExactInt* b,
    const ExactInt* c,
    const ExactInt* d)
{
    const ExactInt ax = a[0] - d[0], ay = a[1] - d[1];
    const ExactInt bx = b[0] - d[0], by = b[1] - d[1];
    const ExactInt cx = c[0] - d[0], cy = c[1] - d[1];
    return det3(
        ax,
        ay,
        ax * ax + ay * ay,
        bx,
        by,
        bx * bx + by * by,
        cx,
        cy,
        cx * cx + cy * cy);
}
/// -( (a - c) . (b - c) ): degree 2, so 2^20 coordinates cannot overflow
ExactInt
indiamcircleInt(const ExactInt* a, const ExactInt* b, const ExactInt* c)
{
    return -((a[0] - c[0]) * (b[0] - c[0]) + (a[1] - c[1]) * (b[1] - c[1]));
}

ExactInt orient3dInt(
    const ExactInt* a,
    const ExactInt* b,
    const ExactInt* c,
    const ExactInt* d)
{
    return det3(
        a[0] - d[0],
        a[1] - d[1],
        a[2] - d[2],
        b[0] - d[0],
        b[1] - d[1],
        b[2] - d[2],
        c[0] - d[0],
        c[1] - d[1],
        c[2] - d[2]);
}

ExactInt insphereInt(
    const ExactInt* a,
    const ExactInt* b,
    const ExactInt* c,
    const ExactInt* d,
    const ExactInt* e)
{
    const ExactInt* p[4] = {a, b, c, d};
    ExactInt m[4][4];
    for(std::size_t i = 0; i < 4; ++i)
    {
        for(std::size_t k = 0; k < 3; ++k)
            m[i][k] = p[i][k] - e[k];
        m[i][3] = m[i][0] * m[i][0] + m[i][1] * m[i][1] + m[i][2] * m[i][2];
    }
    ExactInt result = 0;
    for(std::size_t i = 0; i < 4; ++i) // expand along the "lifted" column
    {
        std::size_t r[3];
        std::size_t n = 0;
        for(std::size_t k = 0; k < 4; ++k)
            if(k != i)
                r[n++] = k;
        const ExactInt minor = det3(
            m[r[0]][0],
            m[r[0]][1],
            m[r[0]][2],
            m[r[1]][0],
            m[r[1]][1],
            m[r[1]][2],
            m[r[2]][0],
            m[r[2]][1],
            m[r[2]][2]);
        result += ((i + 3) % 2 == 0 ? 1 : -1) * m[i][3] * minor;
    }
    return result;
}

/// A point of the integer grids the predicates are swept over
using GridPoint2 = std::array<ExactInt, 2>;

GridPoint2 makeGridPoint(const ExactInt x, const ExactInt y)
{
    GridPoint2 p = {{x, y}};
    return p;
}

/// Small integers are exact in float and double alike, so the integer oracle
/// is ground truth for both
template <typename T, std::size_t N>
void toCoords(const ExactInt* from, T (&to)[N])
{
    for(std::size_t i = 0; i < N; ++i)
        to[i] = static_cast<T>(from[i]);
}

/// One fixture line: a case index, `N` coordinates and the expected sign
template <std::size_t N>
struct Fixture
{
    double c[N];
    int expectedSign;
    int index;
};

/// Fixtures live next to the tests: ctest runs from there, a directly launched
/// test binary does not
std::ifstream openFixtureFile(const std::string& fileName)
{
    std::ifstream f(fileName);
#ifdef CDT_TESTS_DIRECTORY
    if(!f.is_open())
        f.open(std::string(CDT_TESTS_DIRECTORY) + '/' + fileName);
#endif
    if(!f.is_open())
    {
        throw std::runtime_error(
            "Could not open predicate fixture file '" + fileName + '\'');
    }
    return f;
}

template <std::size_t N>
std::vector<Fixture<N> > readFixtures(const std::string& fileName)
{
    std::ifstream f = openFixtureFile(fileName);
    std::vector<Fixture<N> > out;
    std::string line;
    while(std::getline(f, line))
    {
        if(line.empty())
            continue;
        std::istringstream ss(line);
        Fixture<N> fx;
        ss >> fx.index;
        for(std::size_t i = 0; i < N; ++i)
            ss >> fx.c[i];
        ss >> fx.expectedSign;
        if(ss.fail())
            throw std::runtime_error("Malformed fixture line: '" + line + '\'');
        out.push_back(fx);
    }
    if(out.empty())
        throw std::runtime_error("No fixtures read from '" + fileName + '\'');
    return out;
}

/// Random significand scaled by a random power of two, after Shewchuk's
/// `predicates.c` generators: sweeps exponents instead of clustering around 1

template <typename T>
class RandomCoord
{
public:
    /// \param maxExponent scale is 2^[0, maxExponent]; keep it small for
    ///        "narrow" inputs whose products stay exact for longer
    explicit RandomCoord(const int maxExponent)
        : m_significand(T(-1), T(1))
        , m_exponent(0, maxExponent)
    {}

    template <typename TRandomGenerator>
    T operator()(TRandomGenerator& g)
    {
        return std::ldexp(m_significand(g), m_exponent(g));
    }

private:
    std::uniform_real_distribution<T> m_significand;
    std::uniform_int_distribution<int> m_exponent;
};

} // namespace

/* The predicates are built out of error-free transformations: `x = a + b`
 * followed by an expression that recovers the rounding error of that very
 * addition. Re-associating floating-point expressions (`-ffast-math`,
 * `-fassociative-math`, `/fp:fast`, `-Ofast`) turns those into literal zero and
 * every predicate below silently starts returning wrong signs. The canary. */
TEST_CASE(
    "Predicates: the build's floating-point model preserves error-free "
    "transformations",
    "[predicates]")
{
#ifdef __FAST_MATH__
    FAIL(
        "compiled with -ffast-math (or -Ofast): floating-point re-association "
        "breaks the robust predicates");
#endif
#if defined(_M_FP_FAST)
    FAIL("compiled with /fp:fast: this breaks the robust predicates");
#endif

    using Base = predicates::detail::ExpansionBase<double>;

    SECTION("Two-Sum recovers the rounding error of an addition")
    {
        // 1 + 2^-60 rounds back to 1, and the whole of 2^-60 is the roundoff
        volatile double va = 1.0;
        volatile double vb = std::ldexp(1.0, -60);
        const double a = va;
        const double b = vb;
        const double x = a + b;
        REQUIRE(x == a);
        REQUIRE(Base::PlusTail(a, b, x) == b);
        REQUIRE(Base::FastPlusTail(a, b, x) == b);
        REQUIRE(Base::MinusTail(a, -b, x) == b);
    }

    SECTION("Two-Product recovers the rounding error of a multiplication")
    {
        // (1 + 2^-30)^2 == 1 + 2^-29 + 2^-60 needs 61 bits, so it rounds to
        // 1 + 2^-29 and leaves exactly 2^-60 behind
        volatile double vp = 1.0 + std::ldexp(1.0, -30);
        const double p = vp;
        const double x = p * p;
        REQUIRE(x == 1.0 + std::ldexp(1.0, -29));
        REQUIRE(Base::MultTail(p, p, x) == std::ldexp(1.0, -60));
    }

    SECTION("Dekker's split produces non-overlapping halves")
    {
        volatile double vp = 1.0 + std::ldexp(1.0, -30);
        const double p = vp;
        const std::pair<double, double> split = Base::Split(p);
        REQUIRE(split.first == 1.0);
        REQUIRE(split.second == std::ldexp(1.0, -30));
    }
}

/* Integer operands make the error-free transformations checkable without any
 * extended precision: the rounded result and its roundoff are both integers. */
TEST_CASE(
    "Predicates: expansion arithmetic is exact for integer operands",
    "[predicates]")
{
    using Base = predicates::detail::ExpansionBase<double>;

    std::mt19937 gen(20250822);
    // 2^31 keeps a*b inside int64_t and inside double's exponent range
    std::uniform_int_distribution<ExactInt> coord(
        -(ExactInt(1) << 31), (ExactInt(1) << 31) - 1);

    FailureLog plus;
    FailureLog mult;
    for(int i = 0; i < 100000; ++i)
    {
        const ExactInt a = coord(gen);
        const ExactInt b = coord(gen);
        const double da = static_cast<double>(a);
        const double db = static_cast<double>(b);

        const double sum = da + db;
        const double sumTail = Base::PlusTail(da, db, sum);
        if(static_cast<ExactInt>(sum) + static_cast<ExactInt>(sumTail) != a + b)
        {
            std::ostringstream o;
            o << a << " + " << b;
            plus.add(o.str());
        }

        const double product = da * db;
        const double productTail = Base::MultTail(da, db, product);
        if(static_cast<ExactInt>(product) +
               static_cast<ExactInt>(productTail) !=
           a * b)
        {
            std::ostringstream o;
            o << a << " * " << b;
            mult.add(o.str());
        }
    }
    REQUIRE_NO_FAILURES(plus);
    REQUIRE_NO_FAILURES(mult);
}

/* Basic sign cases from mourner/robust-predicates' `test/test.js`. This library
 * returns the determinant as documented in `predicates.h`, which negates what
 * mourner's `orient2d`/`insphere` return. */
TEMPLATE_LIST_TEST_CASE(
    "Predicates: orient2d sign convention",
    "[predicates]",
    CoordTypes)
{
    const TestType a[2] = {TestType(0), TestType(0)};
    const TestType b[2] = {TestType(1), TestType(1)};
    const TestType left[2] = {TestType(0), TestType(1)};
    const TestType right[2] = {TestType(1), TestType(0)};
    const TestType on[2] = {TestType(0.5), TestType(0.5)};

    SECTION("point to the left of a -> b is positive")
    {
        REQUIRE(predicates::detail::exact::orient2d(a, b, left) > TestType(0));
        REQUIRE(predicates::orient2d(a, b, left) > TestType(0));
    }
    SECTION("point to the right of a -> b is negative")
    {
        REQUIRE(predicates::detail::exact::orient2d(a, b, right) < TestType(0));
        REQUIRE(predicates::orient2d(a, b, right) < TestType(0));
    }
    SECTION("collinear point is exactly zero")
    {
        REQUIRE(predicates::detail::exact::orient2d(a, b, on) == TestType(0));
        REQUIRE(predicates::orient2d(a, b, on) == TestType(0));
    }
    SECTION("the scalar and pointer overloads agree")
    {
        REQUIRE(
            predicates::detail::exact::orient2d(a, b, left) ==
            predicates::detail::exact::orient2d(
                a[0], a[1], b[0], b[1], left[0], left[1]));
        REQUIRE(
            predicates::orient2d(a, b, left) ==
            predicates::orient2d(a[0], a[1], b[0], b[1], left[0], left[1]));
    }
    SECTION("degenerate inputs")
    {
        REQUIRE(predicates::detail::exact::orient2d(a, a, b) == TestType(0));
        REQUIRE(predicates::orient2d(a, a, b) == TestType(0));
        REQUIRE(predicates::detail::exact::orient2d(a, b, b) == TestType(0));
        REQUIRE(predicates::orient2d(a, b, b) == TestType(0));
        REQUIRE(predicates::detail::exact::orient2d(a, a, a) == TestType(0));
        REQUIRE(predicates::orient2d(a, a, a) == TestType(0));
    }
}

TEMPLATE_LIST_TEST_CASE(
    "Predicates: incircle sign convention",
    "[predicates]",
    CoordTypes)
{
    // counter-clockwise triangle inscribed in the unit circle
    const TestType a[2] = {TestType(1), TestType(0)};
    const TestType b[2] = {TestType(0), TestType(1)};
    const TestType c[2] = {TestType(-1), TestType(0)};
    const TestType inside[2] = {TestType(0), TestType(-0.5)};
    const TestType on[2] = {TestType(0), TestType(-1)};
    const TestType outside[2] = {TestType(0), TestType(-1.5)};

    SECTION("point inside the circumcircle is positive")
    {
        REQUIRE(
            predicates::detail::exact::incircle(a, b, c, inside) > TestType(0));
        REQUIRE(predicates::incircle(a, b, c, inside) > TestType(0));
    }
    SECTION("cocircular point is exactly zero")
    {
        REQUIRE(
            predicates::detail::exact::incircle(a, b, c, on) == TestType(0));
        REQUIRE(predicates::incircle(a, b, c, on) == TestType(0));
    }
    SECTION("point outside the circumcircle is negative")
    {
        REQUIRE(
            predicates::detail::exact::incircle(a, b, c, outside) <
            TestType(0));
        REQUIRE(predicates::incircle(a, b, c, outside) < TestType(0));
    }
    SECTION("a clockwise triangle flips the sign")
    {
        REQUIRE(
            predicates::detail::exact::incircle(c, b, a, inside) < TestType(0));
        REQUIRE(predicates::incircle(c, b, a, inside) < TestType(0));
    }
    SECTION("the scalar and pointer overloads agree")
    {
        REQUIRE(
            predicates::detail::exact::incircle(a, b, c, inside) ==
            predicates::detail::exact::incircle(
                a[0], a[1], b[0], b[1], c[0], c[1], inside[0], inside[1]));
        REQUIRE(
            predicates::incircle(a, b, c, inside) ==
            predicates::incircle(
                a[0], a[1], b[0], b[1], c[0], c[1], inside[0], inside[1]));
    }
    SECTION("a duplicated point makes the result zero")
    {
        REQUIRE(predicates::detail::exact::incircle(a, b, c, a) == TestType(0));
        REQUIRE(predicates::incircle(a, b, c, a) == TestType(0));
        REQUIRE(
            predicates::detail::exact::incircle(a, a, c, inside) ==
            TestType(0));
        REQUIRE(predicates::incircle(a, a, c, inside) == TestType(0));
    }
}

TEMPLATE_LIST_TEST_CASE(
    "Predicates: orient3d sign convention",
    "[predicates]",
    CoordTypes)
{
    const TestType a[3] = {TestType(0), TestType(0), TestType(0)};
    const TestType b[3] = {TestType(0), TestType(1), TestType(0)};
    const TestType c[3] = {TestType(1), TestType(0), TestType(0)};
    const TestType above[3] = {TestType(0), TestType(0), TestType(1)};
    const TestType below[3] = {TestType(0), TestType(0), TestType(-1)};
    const TestType on[3] = {TestType(0), TestType(0), TestType(0)};

    REQUIRE(predicates::detail::exact::orient3d(a, b, c, above) > TestType(0));
    REQUIRE(predicates::orient3d(a, b, c, above) > TestType(0));
    REQUIRE(predicates::detail::exact::orient3d(a, b, c, below) < TestType(0));
    REQUIRE(predicates::orient3d(a, b, c, below) < TestType(0));
    REQUIRE(predicates::detail::exact::orient3d(a, b, c, on) == TestType(0));
    REQUIRE(predicates::orient3d(a, b, c, on) == TestType(0));

    SECTION("one ULP off the plane is still resolved")
    {
        const TestType up[3] = {0, 0, std::nextafter(TestType(0), TestType(1))};
        const TestType down[3] = {
            0, 0, std::nextafter(TestType(0), TestType(-1))};
        REQUIRE(predicates::detail::exact::orient3d(a, b, c, up) > TestType(0));
        REQUIRE(predicates::orient3d(a, b, c, up) > TestType(0));
        REQUIRE(
            predicates::detail::exact::orient3d(a, b, c, down) < TestType(0));
        REQUIRE(predicates::orient3d(a, b, c, down) < TestType(0));
    }
}

TEMPLATE_LIST_TEST_CASE(
    "Predicates: insphere sign convention",
    "[predicates]",
    CoordTypes)
{
    const TestType a[3] = {TestType(1), TestType(0), TestType(0)};
    const TestType b[3] = {TestType(0), TestType(-1), TestType(0)};
    const TestType c[3] = {TestType(0), TestType(1), TestType(0)};
    const TestType d[3] = {TestType(0), TestType(0), TestType(1)};
    const TestType inside[3] = {TestType(0), TestType(0), TestType(0)};
    const TestType outside[3] = {TestType(0), TestType(0), TestType(2)};
    const TestType on[3] = {TestType(0), TestType(0), TestType(-1)};

    REQUIRE(
        predicates::detail::exact::insphere(a, b, c, d, inside) > TestType(0));
    REQUIRE(predicates::insphere(a, b, c, d, inside) > TestType(0));
    REQUIRE(
        predicates::detail::exact::insphere(a, b, c, d, outside) < TestType(0));
    REQUIRE(predicates::insphere(a, b, c, d, outside) < TestType(0));
    REQUIRE(predicates::detail::exact::insphere(a, b, c, d, on) == TestType(0));
    REQUIRE(predicates::insphere(a, b, c, d, on) == TestType(0));

    SECTION("one ULP off the sphere is still resolved")
    {
        const TestType in[3] = {
            0, 0, std::nextafter(TestType(-1), TestType(0))};
        const TestType out[3] = {
            0, 0, std::nextafter(TestType(-1), TestType(-2))};
        REQUIRE(
            predicates::detail::exact::insphere(a, b, c, d, in) > TestType(0));
        REQUIRE(predicates::insphere(a, b, c, d, in) > TestType(0));
        REQUIRE(
            predicates::detail::exact::insphere(a, b, c, d, out) < TestType(0));
        REQUIRE(predicates::insphere(a, b, c, d, out) < TestType(0));
    }
}
/* `indiamcircle(a, b, c)` reports where `c` lies relative to the circle that
 * has `ab` as its diameter, using incircle's sign convention. The two ends of
 * the diameter and every right angle over it lie exactly on that circle. */
TEMPLATE_LIST_TEST_CASE(
    "Predicates: indiamcircle sign convention",
    "[predicates]",
    CoordTypes)
{
    // circle with diameter ab: centre (2, 0), radius 2
    const TestType a[2] = {TestType(0), TestType(0)};
    const TestType b[2] = {TestType(4), TestType(0)};
    const TestType inside[2] = {TestType(2), TestType(1)};
    const TestType on[2] = {TestType(2), TestType(2)};
    const TestType outside[2] = {TestType(2), TestType(3)};

    SECTION("point inside the diametral circle is positive")
    {
        REQUIRE(
            predicates::detail::exact::indiamcircle(a, b, inside) >
            TestType(0));
        REQUIRE(predicates::indiamcircle(a, b, inside) > TestType(0));
    }
    SECTION("point on the diametral circle is exactly zero")
    {
        REQUIRE(
            predicates::detail::exact::indiamcircle(a, b, on) == TestType(0));
        REQUIRE(predicates::indiamcircle(a, b, on) == TestType(0));
    }
    SECTION("point outside the diametral circle is negative")
    {
        REQUIRE(
            predicates::detail::exact::indiamcircle(a, b, outside) <
            TestType(0));
        REQUIRE(predicates::indiamcircle(a, b, outside) < TestType(0));
    }
    SECTION("the diameter's own end points lie on the circle")
    {
        REQUIRE(
            predicates::detail::exact::indiamcircle(a, b, a) == TestType(0));
        REQUIRE(predicates::indiamcircle(a, b, a) == TestType(0));
        REQUIRE(
            predicates::detail::exact::indiamcircle(a, b, b) == TestType(0));
        REQUIRE(predicates::indiamcircle(a, b, b) == TestType(0));
    }
    SECTION("swapping the diameter's ends does not change the result")
    {
        REQUIRE(
            predicates::indiamcircle(a, b, inside) ==
            predicates::indiamcircle(b, a, inside));
        REQUIRE(
            predicates::detail::exact::indiamcircle(a, b, outside) ==
            predicates::detail::exact::indiamcircle(b, a, outside));
    }
    SECTION("the scalar and pointer overloads agree")
    {
        REQUIRE(
            predicates::indiamcircle(a, b, inside) ==
            predicates::indiamcircle(
                a[0], a[1], b[0], b[1], inside[0], inside[1]));
        REQUIRE(
            predicates::detail::exact::indiamcircle(a, b, inside) ==
            predicates::detail::exact::indiamcircle(
                a[0], a[1], b[0], b[1], inside[0], inside[1]));
    }
    SECTION("a degenerate diameter leaves only its own end point on the circle")
    {
        REQUIRE(
            predicates::detail::exact::indiamcircle(a, a, a) == TestType(0));
        REQUIRE(predicates::indiamcircle(a, a, a) == TestType(0));
        REQUIRE(predicates::detail::exact::indiamcircle(a, a, b) < TestType(0));
        REQUIRE(predicates::indiamcircle(a, a, b) < TestType(0));
    }
    SECTION("every right angle over the diameter is exactly on the circle")
    {
        // Thales: integer right triangles on the circle of diameter
        // (0,0)-(10,0)
        const TestType from[2] = {TestType(0), TestType(0)};
        const TestType to[2] = {TestType(10), TestType(0)};
        const TestType rightAngles[6][2] = {
            {TestType(1), TestType(3)},
            {TestType(9), TestType(3)},
            {TestType(2), TestType(4)},
            {TestType(8), TestType(4)},
            {TestType(5), TestType(5)},
            {TestType(2), TestType(-4)}};
        for(std::size_t i = 0; i < 6; ++i)
        {
            INFO("right angle #" << i);
            REQUIRE(
                predicates::indiamcircle(from, to, rightAngles[i]) ==
                TestType(0));
            REQUIRE(
                predicates::detail::exact::indiamcircle(
                    from, to, rightAngles[i]) == TestType(0));
        }
    }
}

/* The predicates are determinants whose rows are the input points, so swapping
 * two arguments must negate the result. Only the sign is checked: `exact`
 * returns the most significant expansion component rather than a rounded sum,
 * and a permuted argument order reaches the same value along a different
 * sequence of operations. */
TEST_CASE(
    "Predicates: swapping two arguments negates the result",
    "[predicates]")
{
    std::mt19937 gen(20250823);
    RandomCoord<double> coord(24);

    FailureLog log;
    for(int iter = 0; iter < 200; ++iter)
    {
        double p[5][3];
        for(std::size_t i = 0; i < 5; ++i)
            for(std::size_t k = 0; k < 3; ++k)
                p[i][k] = coord(gen);

        for(std::size_t i = 0; i < 5; ++i)
        {
            for(std::size_t j = i + 1; j < 5; ++j)
            {
                const double* q[5] = {p[0], p[1], p[2], p[3], p[4]};
                std::swap(q[i], q[j]);

                std::ostringstream what;
                what << "iteration " << iter << ", swap " << i << "<->" << j;

                if(j < 3 &&
                   signOf(
                       predicates::detail::exact::orient2d(q[0], q[1], q[2])) !=
                       -signOf(
                           predicates::detail::exact::orient2d(
                               p[0], p[1], p[2])))
                    log.add("orient2d, " + what.str());
                if(j < 4 && signOf(
                                predicates::detail::exact::incircle(
                                    q[0], q[1], q[2], q[3])) !=
                                -signOf(
                                    predicates::detail::exact::incircle(
                                        p[0], p[1], p[2], p[3])))
                    log.add("incircle, " + what.str());
                if(j < 4 && signOf(
                                predicates::detail::exact::orient3d(
                                    q[0], q[1], q[2], q[3])) !=
                                -signOf(
                                    predicates::detail::exact::orient3d(
                                        p[0], p[1], p[2], p[3])))
                    log.add("orient3d, " + what.str());
                if(signOf(
                       predicates::detail::exact::insphere(
                           q[0], q[1], q[2], q[3], q[4])) !=
                   -signOf(
                       predicates::detail::exact::insphere(
                           p[0], p[1], p[2], p[3], p[4])))
                    log.add("insphere, " + what.str());
            }
        }
    }
    REQUIRE_NO_FAILURES(log);
}

/* Small integer grids are dense in the degeneracies where a floating-point
 * determinant goes wrong; every point combination is swept. */
TEMPLATE_LIST_TEST_CASE(
    "Predicates: orient2d matches an exact integer oracle on a small grid",
    "[predicates]",
    CoordTypes)
{
    std::vector<GridPoint2> grid;
    for(ExactInt x = -2; x <= 2; ++x)
        for(ExactInt y = -2; y <= 2; ++y)
            grid.push_back(makeGridPoint(x, y));
    const std::size_t n = grid.size();

    FailureLog log;
    std::size_t degenerate = 0;
    for(std::size_t i = 0; i < n; ++i)
    {
        for(std::size_t j = 0; j < n; ++j)
        {
            for(std::size_t k = 0; k < n; ++k)
            {
                const ExactInt* a = grid[i].data();
                const ExactInt* b = grid[j].data();
                const ExactInt* c = grid[k].data();
                const int expected = signOf(orient2dInt(a, b, c));
                if(expected == 0)
                    ++degenerate;

                TestType fa[2], fb[2], fc[2];
                toCoords(a, fa);
                toCoords(b, fb);
                toCoords(c, fc);
                const int gotExact =
                    signOf(predicates::detail::exact::orient2d(fa, fb, fc));
                const int gotAdaptive =
                    signOf(predicates::orient2d(fa, fb, fc));
                if(gotExact != expected || gotAdaptive != expected)
                {
                    std::ostringstream o;
                    o << '(' << a[0] << ',' << a[1] << ") (" << b[0] << ','
                      << b[1] << ") (" << c[0] << ',' << c[1] << "): expected "
                      << expected << ", exact " << gotExact << ", adaptive "
                      << gotAdaptive;
                    log.add(o.str());
                }
            }
        }
    }
    REQUIRE_NO_FAILURES(log);
    REQUIRE(degenerate > 0); // the grid really does exercise collinear triples
}

TEMPLATE_LIST_TEST_CASE(
    "Predicates: incircle matches an exact integer oracle on a small grid",
    "[predicates]",
    CoordTypes)
{
    std::vector<GridPoint2> grid;
    for(ExactInt x = -1; x <= 2; ++x)
        for(ExactInt y = -1; y <= 2; ++y)
            grid.push_back(makeGridPoint(x, y));
    const std::size_t n = grid.size();

    FailureLog log;
    std::size_t degenerate = 0;
    for(std::size_t i = 0; i < n; ++i)
    {
        for(std::size_t j = 0; j < n; ++j)
        {
            for(std::size_t k = 0; k < n; ++k)
            {
                for(std::size_t l = 0; l < n; ++l)
                {
                    const ExactInt* a = grid[i].data();
                    const ExactInt* b = grid[j].data();
                    const ExactInt* c = grid[k].data();
                    const ExactInt* d = grid[l].data();
                    const int expected = signOf(incircleInt(a, b, c, d));
                    if(expected == 0)
                        ++degenerate;

                    TestType fa[2], fb[2], fc[2], fd[2];
                    toCoords(a, fa);
                    toCoords(b, fb);
                    toCoords(c, fc);
                    toCoords(d, fd);
                    const int gotExact = signOf(
                        predicates::detail::exact::incircle(fa, fb, fc, fd));
                    const int gotAdaptive =
                        signOf(predicates::incircle(fa, fb, fc, fd));
                    if(gotExact != expected || gotAdaptive != expected)
                    {
                        std::ostringstream o;
                        o << '(' << a[0] << ',' << a[1] << ") (" << b[0] << ','
                          << b[1] << ") (" << c[0] << ',' << c[1] << ") ("
                          << d[0] << ',' << d[1] << "): expected " << expected
                          << ", exact " << gotExact << ", adaptive "
                          << gotAdaptive;
                        log.add(o.str());
                    }
                }
            }
        }
    }
    REQUIRE_NO_FAILURES(log);
    REQUIRE(degenerate > 0); // the grid really does exercise cocircular points
}

/* 3D counterpart of the grid sweeps above, which would be too large to run
 * exhaustively: a tiny integer alphabet still makes many cases degenerate. */
/* Grid counterpart of the orient2d and incircle sweeps above. */
TEMPLATE_LIST_TEST_CASE(
    "Predicates: indiamcircle matches an exact integer oracle on a small grid",
    "[predicates]",
    CoordTypes)
{
    std::vector<GridPoint2> grid;
    for(ExactInt x = -2; x <= 2; ++x)
        for(ExactInt y = -2; y <= 2; ++y)
            grid.push_back(makeGridPoint(x, y));
    const std::size_t n = grid.size();

    FailureLog log;
    std::size_t degenerate = 0;
    for(std::size_t i = 0; i < n; ++i)
    {
        for(std::size_t j = 0; j < n; ++j)
        {
            for(std::size_t k = 0; k < n; ++k)
            {
                const ExactInt* a = grid[i].data();
                const ExactInt* b = grid[j].data();
                const ExactInt* c = grid[k].data();
                const int expected = signOf(indiamcircleInt(a, b, c));
                if(expected == 0)
                    ++degenerate;

                TestType fa[2], fb[2], fc[2];
                toCoords(a, fa);
                toCoords(b, fb);
                toCoords(c, fc);
                const int gotExact =
                    signOf(predicates::detail::exact::indiamcircle(fa, fb, fc));
                const int gotAdaptive =
                    signOf(predicates::indiamcircle(fa, fb, fc));
                if(gotExact != expected || gotAdaptive != expected)
                {
                    std::ostringstream o;
                    o << '(' << a[0] << ',' << a[1] << ") (" << b[0] << ','
                      << b[1] << ") (" << c[0] << ',' << c[1] << "): expected "
                      << expected << ", exact " << gotExact << ", adaptive "
                      << gotAdaptive;
                    log.add(o.str());
                }
            }
        }
    }
    REQUIRE_NO_FAILURES(log);
    REQUIRE(degenerate > 0); // the grid really does hit points on the circle
}

/* `c` is placed on the diametral circle of `ab` in inexact floating point: the
 * dot product then cancels catastrophically, which is where the naive
 * expression this predicate replaces gets the sign wrong. The naive count is
 * asserted to be non-zero so that the case generator cannot silently stop
 * producing hard input. */
TEST_CASE(
    "Predicates: indiamcircle on points placed on the diametral circle",
    "[predicates]")
{
    std::mt19937 gen(20250901);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    std::uniform_real_distribution<double> angle(0.0, 6.283185307179586);

    FailureLog log;
    std::size_t naiveWrong = 0;
    for(int iter = 0; iter < 20000; ++iter)
    {
        const double scale = std::ldexp(1.0, iter % 40 - 20);
        const double a[2] = {unit(gen) * scale, unit(gen) * scale};
        const double b[2] = {unit(gen) * scale, unit(gen) * scale};
        const double midX = (a[0] + b[0]) / 2, midY = (a[1] + b[1]) / 2;
        const double radius =
            std::sqrt(
                (b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1])) /
            2;
        const double phi = angle(gen);
        const double c[2] = {
            midX + radius * std::cos(phi), midY + radius * std::sin(phi)};

        const int expected =
            signOf(predicates::detail::exact::indiamcircle(a, b, c));
        if(signOf(predicates::indiamcircle(a, b, c)) != expected)
            log.add("iteration " + std::to_string(iter));

        const double naive =
            -((a[0] - c[0]) * (b[0] - c[0]) + (a[1] - c[1]) * (b[1] - c[1]));
        if(signOf(naive) != expected)
            ++naiveWrong;
    }
    REQUIRE_NO_FAILURES(log);
    INFO("naive dot product got " << naiveWrong << " of 20000 signs wrong");
    REQUIRE(naiveWrong > 0);
}

/* Input that exhausts every filter and reaches the final expansion.
 *
 * Reaching it needs the determinant to be zero while the coordinate
 * differences still round, so that neither the cheap products, nor the
 * expansion of the rounded differences, nor the round-off correction can
 * settle the sign. Two families, both of which cancel exactly:
 *   symmetric   a = (s, s),   c = (e, e),   b = (e + d, e - d)
 *   asymmetric  a = (2s, s),  c = (2e, e),  b = (2e + d, e - 2d)
 * Whether `s - e` is representable depends on how far apart the exponents are,
 * so only part of the sweep reaches the last expansion; the rest still has to
 * come out exactly zero, from an earlier filter. */
TEST_CASE(
    "Predicates: indiamcircle on input that reaches the final expansion",
    "[predicates]")
{
    typedef predicates::detail::ExpansionBase<double> Base;

    FailureLog log;
    std::size_t roundedDifferences = 0;
    for(int se = -6; se <= 6; ++se)
    {
        const double s = std::ldexp(1.0, se);
        for(int ee = -70; ee < -40; ee += 3)
        {
            const double e = std::ldexp(1.0, ee);
            for(int de = -80; de < -55; de += 3)
            {
                const double d = std::ldexp(1.0, de);
                if(e + d == e || e - d == e)
                    continue;

                const double sym[3][2] = {{s, s}, {e + d, e - d}, {e, e}};
                const double asym[3][2] = {
                    {2 * s, s}, {2 * e + d, e - 2 * d}, {2 * e, e}};
                const double(*families[2])[2] = {sym, asym};
                for(std::size_t f = 0; f < 2; ++f)
                {
                    const double* a = families[f][0];
                    const double* b = families[f][1];
                    const double* c = families[f][2];

                    // a rounding difference is what denies the cheaper filters
                    // an exact answer and forces the final expansion
                    if(Base::MinusTail(a[0], c[0], a[0] - c[0]) != 0.0)
                        ++roundedDifferences;

                    if(predicates::indiamcircle(a, b, c) != 0.0 ||
                       predicates::detail::exact::indiamcircle(a, b, c) != 0.0)
                    {
                        std::ostringstream o;
                        o << "family " << f << " s=2^" << se << " e=2^" << ee
                          << " d=2^" << de;
                        log.add(o.str());
                    }
                }
            }
        }
    }
    REQUIRE_NO_FAILURES(log);
    REQUIRE(roundedDifferences > 100);
}

TEST_CASE(
    "Predicates: orient3d and insphere match an exact integer oracle",
    "[predicates]")
{
    std::mt19937 gen(20250824);
    std::uniform_int_distribution<ExactInt> coord(-3, 3);

    FailureLog log;
    std::size_t coplanar = 0;
    std::size_t cospherical = 0;
    for(int iter = 0; iter < 20000; ++iter)
    {
        ExactInt p[5][3];
        for(std::size_t i = 0; i < 5; ++i)
            for(std::size_t k = 0; k < 3; ++k)
                p[i][k] = coord(gen);

        double a[3], b[3], c[3], d[3], e[3];
        toCoords(p[0], a);
        toCoords(p[1], b);
        toCoords(p[2], c);
        toCoords(p[3], d);
        toCoords(p[4], e);

        const int expected3d = signOf(orient3dInt(p[0], p[1], p[2], p[3]));
        if(expected3d == 0)
            ++coplanar;
        if(signOf(predicates::detail::exact::orient3d(a, b, c, d)) !=
               expected3d ||
           signOf(predicates::orient3d(a, b, c, d)) != expected3d)
        {
            std::ostringstream o;
            o << "orient3d, iteration " << iter << ", expected " << expected3d;
            log.add(o.str());
        }

        const int expectedSphere =
            signOf(insphereInt(p[0], p[1], p[2], p[3], p[4]));
        if(expectedSphere == 0)
            ++cospherical;
        if(signOf(predicates::detail::exact::insphere(a, b, c, d, e)) !=
               expectedSphere ||
           signOf(predicates::insphere(a, b, c, d, e)) != expectedSphere)
        {
            std::ostringstream o;
            o << "insphere, iteration " << iter << ", expected "
              << expectedSphere;
            log.add(o.str());
        }
    }
    REQUIRE_NO_FAILURES(log);
    REQUIRE(coplanar > 0);
    REQUIRE(cospherical > 0);
}

/* From mourner/robust-predicates: query points packed into a few ULPs, against
 * a line through two far-away points. They all round to the same naive
 * determinant, so only exact arithmetic can tell them apart. */
TEMPLATE_LIST_TEST_CASE(
    "Predicates: orient2d on a near-collinear ULP-scale grid",
    "[predicates]",
    CoordTypes)
{
    const TestType r = TestType(0.95);
    const TestType q = TestType(18);
    const TestType p = TestType(16.8);
    // the grid spans ten binades below the ULP of `r`
    const TestType w =
        std::ldexp(TestType(1), 10 - std::numeric_limits<TestType>::digits);

    FailureLog log;
    for(int i = 0; i < 128; ++i)
    {
        for(int j = 0; j < 128; ++j)
        {
            const TestType x = r + w * TestType(i) / TestType(128);
            const TestType y = r + w * TestType(j) / TestType(128);
            const int expected =
                signOf(predicates::detail::exact::orient2d(x, y, q, q, p, p));
            const int got = signOf(predicates::orient2d(x, y, q, q, p, p));
            if(got != expected)
            {
                std::ostringstream o;
                o << "i=" << i << ", j=" << j << ": expected " << expected
                  << ", got " << got;
                log.add(o.str());
            }
        }
    }
    REQUIRE_NO_FAILURES(log);
}

/* From mourner/robust-predicates: one cocircular configuration swept across the
 * whole exponent range, catching bounds that only hold for inputs near 1. */
TEST_CASE(
    "Predicates: incircle over the full range of coordinate magnitudes",
    "[predicates]")
{
    FailureLog log;
    double x = 1e-64;
    for(int i = 0; i < 128; ++i)
    {
        // counter-clockwise, circumcircle centred at (0, -x/4) of radius 5x/4
        const double a[2] = {0, x};
        const double b[2] = {-x, -x};
        const double c[2] = {x, -x};
        const double inside[2] = {0, 0};
        const double outside[2] = {0, 2 * x};
        const double on[2] = {0, x};

        std::ostringstream at;
        at << " at x=" << x;

        if(!(predicates::incircle(a, b, c, inside) > 0) ||
           !(predicates::detail::exact::incircle(a, b, c, inside) > 0))
            log.add("inside" + at.str());
        if(!(predicates::incircle(a, b, c, outside) < 0) ||
           !(predicates::detail::exact::incircle(a, b, c, outside) < 0))
            log.add("outside" + at.str());
        if(predicates::incircle(a, b, c, on) != 0 ||
           predicates::detail::exact::incircle(a, b, c, on) != 0)
            log.add("cocircular" + at.str());

        x *= 10;
    }
    REQUIRE_NO_FAILURES(log);
}

/* From mourner/robust-predicates: `b`, `c` and `d` are collinear, so the
 * result is exactly zero however the fourth point is jittered. */
TEST_CASE(
    "Predicates: orient3d on a degenerate collinear plane",
    "[predicates]")
{
    std::mt19937 gen(20250825);
    std::uniform_real_distribution<double> jitter(0.0, 5.0e-14);

    FailureLog log;
    for(int i = 0; i < 1000; ++i)
    {
        const double a[3] = {
            0.5 + jitter(gen), 0.5 + jitter(gen), 0.5 + jitter(gen)};
        const double b[3] = {12, 12, 12};
        const double c[3] = {24, 24, 24};
        const double d[3] = {48, 48, 48};

        if(predicates::orient3d(b, c, d, a) != 0 ||
           predicates::detail::exact::orient3d(b, c, d, a) != 0 ||
           predicates::orient3d(c, d, a, b) != 0 ||
           predicates::detail::exact::orient3d(c, d, a, b) != 0)
        {
            log.add("iteration " + std::to_string(i));
        }
    }
    REQUIRE_NO_FAILURES(log);
}

/* A fast filter must never claim a sign the exact computation disagrees with.
 * `c` is placed on the line `ab` and `d` on the circle through `a`, `b`, `c`,
 * both in inexact floating point: where the error bounds are most pressured. */
TEST_CASE(
    "Predicates: adaptive filters agree with exact arithmetic on "
    "near-degenerate input",
    "[predicates]")
{
    std::mt19937 gen(20250826);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    std::uniform_real_distribution<double> angle(0.0, 6.283185307179586);

    FailureLog collinear;
    FailureLog cocircular;
    for(int iter = 0; iter < 20000; ++iter)
    {
        const double scale = std::ldexp(1.0, iter % 40 - 20);
        const double a[2] = {unit(gen) * scale, unit(gen) * scale};
        const double b[2] = {unit(gen) * scale, unit(gen) * scale};
        const double t = unit(gen);
        const double c[2] = {
            a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])};

        if(signOf(predicates::orient2d(a, b, c)) !=
           signOf(predicates::detail::exact::orient2d(a, b, c)))
        {
            collinear.add("iteration " + std::to_string(iter));
        }

        // three more points of the circle centred at `a` through `b`
        const double radius = std::sqrt(
            (b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]));
        double circle[3][2];
        for(std::size_t i = 0; i < 3; ++i)
        {
            const double phi = angle(gen);
            circle[i][0] = a[0] + radius * std::cos(phi);
            circle[i][1] = a[1] + radius * std::sin(phi);
        }
        if(signOf(predicates::incircle(circle[0], circle[1], circle[2], b)) !=
           signOf(
               predicates::detail::exact::incircle(
                   circle[0], circle[1], circle[2], b)))
        {
            cocircular.add("iteration " + std::to_string(iter));
        }
    }
    REQUIRE_NO_FAILURES(collinear);
    REQUIRE_NO_FAILURES(cocircular);
}

/* Shewchuk's own methodology from `predicates.c`: random significands scaled by
 * random powers of two, sweeping many exponents at once. */
TEST_CASE(
    "Predicates: adaptive filters agree with exact arithmetic on random input",
    "[predicates]")
{
    std::mt19937 gen(20250827);

    // narrow exponents keep products exact for longer, wide ones stress the
    // error bounds
    const int exponentRanges[] = {0, 7, 40};

    FailureLog log;
    for(std::size_t e = 0; e < sizeof(exponentRanges) / sizeof(int); ++e)
    {
        RandomCoord<double> coord(exponentRanges[e]);
        for(int iter = 0; iter < 3000; ++iter)
        {
            double p[5][3];
            for(std::size_t i = 0; i < 5; ++i)
                for(std::size_t k = 0; k < 3; ++k)
                    p[i][k] = coord(gen);

            std::ostringstream what;
            what << "exponent range " << exponentRanges[e] << ", iteration "
                 << iter;

            if(signOf(predicates::orient2d(p[0], p[1], p[2])) !=
               signOf(predicates::detail::exact::orient2d(p[0], p[1], p[2])))
                log.add("orient2d, " + what.str());
            if(signOf(predicates::incircle(p[0], p[1], p[2], p[3])) !=
               signOf(
                   predicates::detail::exact::incircle(p[0], p[1], p[2], p[3])))
                log.add("incircle, " + what.str());
            if(signOf(predicates::orient3d(p[0], p[1], p[2], p[3])) !=
               signOf(
                   predicates::detail::exact::orient3d(p[0], p[1], p[2], p[3])))
                log.add("orient3d, " + what.str());
            if(signOf(predicates::insphere(p[0], p[1], p[2], p[3], p[4])) !=
               signOf(
                   predicates::detail::exact::insphere(
                       p[0], p[1], p[2], p[3], p[4])))
                log.add("insphere, " + what.str());
        }
    }
    REQUIRE_NO_FAILURES(log);
}

/* The exact determinant of representable inputs cannot depend on the
 * coordinate type, so `double` is a ground truth for `float`. */
TEST_CASE(
    "Predicates: float results agree with double on representable input",
    "[predicates]")
{
    std::mt19937 gen(20250828);
    RandomCoord<float> coord(20);

    FailureLog log;
    for(int iter = 0; iter < 10000; ++iter)
    {
        float f[5][3];
        double d[5][3];
        for(std::size_t i = 0; i < 5; ++i)
        {
            for(std::size_t k = 0; k < 3; ++k)
            {
                f[i][k] = coord(gen);
                d[i][k] = f[i][k];
            }
        }

        std::ostringstream what;
        what << "iteration " << iter;

        if(signOf(predicates::orient2d(f[0], f[1], f[2])) !=
           signOf(predicates::detail::exact::orient2d(d[0], d[1], d[2])))
            log.add("orient2d, " + what.str());
        if(signOf(predicates::incircle(f[0], f[1], f[2], f[3])) !=
           signOf(predicates::detail::exact::incircle(d[0], d[1], d[2], d[3])))
            log.add("incircle, " + what.str());
        if(signOf(predicates::orient3d(f[0], f[1], f[2], f[3])) !=
           signOf(predicates::detail::exact::orient3d(d[0], d[1], d[2], d[3])))
            log.add("orient3d, " + what.str());
        if(signOf(predicates::insphere(f[0], f[1], f[2], f[3], f[4])) !=
           signOf(
               predicates::detail::exact::insphere(
                   d[0], d[1], d[2], d[3], d[4])))
            log.add("insphere, " + what.str());
    }
    REQUIRE_NO_FAILURES(log);
}

/* From georust/robust: smallest normal coordinates, where the naive
 * determinant underflows to zero but the true sign is well defined. */
TEST_CASE("Predicates: smallest normal coordinates", "[predicates]")
{
    const double tiny = std::numeric_limits<double>::min();

    SECTION("orient2d")
    {
        const double from[2] = {-1, -1};
        const double to[2] = {1, 1};
        const double onLine[2][2] = {{tiny, tiny}, {-tiny, -tiny}};
        const double left[2] = {-tiny, tiny};
        const double right[2] = {tiny, -tiny};

        for(std::size_t i = 0; i < 2; ++i)
        {
            REQUIRE(
                predicates::detail::exact::orient2d(from, to, onLine[i]) == 0);
            REQUIRE(predicates::orient2d(from, to, onLine[i]) == 0);
        }
        REQUIRE(predicates::detail::exact::orient2d(from, to, left) > 0);
        REQUIRE(predicates::orient2d(from, to, left) > 0);
        REQUIRE(predicates::detail::exact::orient2d(from, to, right) < 0);
        REQUIRE(predicates::orient2d(from, to, right) < 0);
    }

    SECTION("orient3d")
    {
        const double a[3] = {1, 0, 1};
        const double b[3] = {-1, 0, -1};
        const double c[3] = {-1, 0, 0};
        const double below[3] = {tiny, tiny, tiny};
        const double above[3] = {-tiny, -tiny, -tiny};
        const double on[3] = {0, 0, 0};

        REQUIRE(predicates::detail::exact::orient3d(a, b, c, below) < 0);
        REQUIRE(predicates::orient3d(a, b, c, below) < 0);
        REQUIRE(predicates::detail::exact::orient3d(a, b, c, above) > 0);
        REQUIRE(predicates::orient3d(a, b, c, above) > 0);
        REQUIRE(predicates::detail::exact::orient3d(a, b, c, on) == 0);
        REQUIRE(predicates::orient3d(a, b, c, on) == 0);
    }

    SECTION("incircle")
    {
        const double from[2] = {-1, -1};
        const double to[2] = {1, 1};
        const double left[2] = {-tiny, tiny};
        const double right[2] = {tiny, -tiny};
        const double query[2] = {2, 2};

        REQUIRE(predicates::detail::exact::incircle(from, left, to, query) > 0);
        REQUIRE(predicates::incircle(from, left, to, query) > 0);
        REQUIRE(
            predicates::detail::exact::incircle(from, to, right, query) > 0);
        REQUIRE(predicates::incircle(from, to, right, query) > 0);
    }
}

/* From georust/robust issue #48: incircle is around 1e-30 here while a broken
 * implementation returns around 1e-16, with the wrong sign in the first case.
 * `exact` returns the most significant expansion component rather than a
 * rounded sum, so only the magnitude is compared to `predicates.c`. */
TEST_CASE("Predicates: regression, georust/robust issue #48", "[predicates]")
{
    SECTION("case a")
    {
        const double a[2] = {2.1045541600524288e-15, -1.0000000000000016};
        const double b[2] = {1.000000000000005, -3.350874324301223e-16};
        const double c[2] = {7.553997323229233e-15, 0.9999999999999958};
        const double d[2] = {-0.9999999999999922, -7.073397829693697e-15};
        const double expected = -8.0140565430358e-30; // from predicates.c

        REQUIRE(predicates::detail::exact::incircle(a, b, c, d) < 0);
        REQUIRE(predicates::incircle(a, b, c, d) < 0);
        REQUIRE(
            std::abs(predicates::incircle(a, b, c, d) - expected) <
            std::abs(expected));
    }

    SECTION("case b")
    {
        const double a[2] = {9.128561612013288e-15, -1.0000000000000029};
        const double b[2] = {1.0000000000000044, -5.451395142523081e-15};
        const double c[2] = {3.851214418148064e-15, 0.9999999999999961};
        const double d[2] = {-0.9999999999999946, -6.6797960341085084e-15};
        const double expected = 7.226864249343135e-30; // from predicates.c

        REQUIRE(predicates::detail::exact::incircle(a, b, c, d) > 0);
        REQUIRE(predicates::incircle(a, b, c, d) > 0);
        REQUIRE(
            std::abs(predicates::incircle(a, b, c, d) - expected) <
            std::abs(expected));
    }
}

/* 1000 near-degenerate cases per predicate, spanning the full exponent range.
 * See inputs/predicates/README.md for their provenance. */

TEST_CASE("Predicates: orient2d hard-case fixtures", "[predicates]")
{
    const std::vector<Fixture<6> > fixtures =
        readFixtures<6>("inputs/predicates/orient2d.txt");
    REQUIRE(fixtures.size() == 1000);

    FailureLog log;
    for(std::size_t i = 0; i < fixtures.size(); ++i)
    {
        const double* c = fixtures[i].c;
        const int expected = fixtures[i].expectedSign;
        const int gotExact =
            signOf(predicates::detail::exact::orient2d(c, c + 2, c + 4));
        const int gotAdaptive = signOf(predicates::orient2d(c, c + 2, c + 4));
        if(gotExact != expected || gotAdaptive != expected)
        {
            std::ostringstream o;
            o << "case " << fixtures[i].index << ": expected " << expected
              << ", exact " << gotExact << ", adaptive " << gotAdaptive;
            log.add(o.str());
        }
    }
    REQUIRE_NO_FAILURES(log);
}

TEST_CASE("Predicates: incircle hard-case fixtures", "[predicates]")
{
    const std::vector<Fixture<8> > fixtures =
        readFixtures<8>("inputs/predicates/incircle.txt");
    REQUIRE(fixtures.size() == 1000);

    FailureLog log;
    for(std::size_t i = 0; i < fixtures.size(); ++i)
    {
        const double* c = fixtures[i].c;
        const int expected = fixtures[i].expectedSign;
        const int gotExact =
            signOf(predicates::detail::exact::incircle(c, c + 2, c + 4, c + 6));
        const int gotAdaptive =
            signOf(predicates::incircle(c, c + 2, c + 4, c + 6));
        if(gotExact != expected || gotAdaptive != expected)
        {
            std::ostringstream o;
            o << "case " << fixtures[i].index << ": expected " << expected
              << ", exact " << gotExact << ", adaptive " << gotAdaptive;
            log.add(o.str());
        }
    }
    REQUIRE_NO_FAILURES(log);
}

TEST_CASE("Predicates: orient3d hard-case fixtures", "[predicates]")
{
    const std::vector<Fixture<12> > fixtures =
        readFixtures<12>("inputs/predicates/orient3d.txt");
    REQUIRE(fixtures.size() == 1000);

    FailureLog log;
    for(std::size_t i = 0; i < fixtures.size(); ++i)
    {
        const double* c = fixtures[i].c;
        const int expected = fixtures[i].expectedSign;
        const int gotExact =
            signOf(predicates::detail::exact::orient3d(c, c + 3, c + 6, c + 9));
        const int gotAdaptive =
            signOf(predicates::orient3d(c, c + 3, c + 6, c + 9));
        if(gotExact != expected || gotAdaptive != expected)
        {
            std::ostringstream o;
            o << "case " << fixtures[i].index << ": expected " << expected
              << ", exact " << gotExact << ", adaptive " << gotAdaptive;
            log.add(o.str());
        }
    }
    REQUIRE_NO_FAILURES(log);
}

TEST_CASE("Predicates: insphere hard-case fixtures", "[predicates]")
{
    const std::vector<Fixture<15> > fixtures =
        readFixtures<15>("inputs/predicates/insphere.txt");
    REQUIRE(fixtures.size() == 1000);

    FailureLog log;
    for(std::size_t i = 0; i < fixtures.size(); ++i)
    {
        const double* c = fixtures[i].c;
        const int expected = fixtures[i].expectedSign;
        const int gotExact = signOf(
            predicates::detail::exact::insphere(
                c, c + 3, c + 6, c + 9, c + 12));
        const int gotAdaptive =
            signOf(predicates::insphere(c, c + 3, c + 6, c + 9, c + 12));
        if(gotExact != expected || gotAdaptive != expected)
        {
            std::ostringstream o;
            o << "case " << fixtures[i].index << ": expected " << expected
              << ", exact " << gotExact << ", adaptive " << gotAdaptive;
            log.add(o.str());
        }
    }
    REQUIRE_NO_FAILURES(log);
}
