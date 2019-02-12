#ifndef CDT_o2ROLeC4NKchgja4Awtg
#define CDT_o2ROLeC4NKchgja4Awtg

#include "CDTUtils.h"

#include <boost/geometry/index/rtree.hpp>

// Specialize boost::geometry point traits for CDT's 2D vector type
namespace boost
{
namespace geometry
{
namespace traits
{

// point traits
template <typename T>
struct tag<CDT::V2d<T> >
{
    typedef point_tag type;
};

template <typename T>
struct dimension<CDT::V2d<T> > : boost::mpl::int_<2>
{};

template <typename T>
struct coordinate_type<CDT::V2d<T> >
{
    typedef T type;
};

template <typename T>
struct coordinate_system<CDT::V2d<T> >
{
    typedef cs::cartesian type;
};

// point access
// X
template <typename T>
struct access<CDT::V2d<T>, 0>
{
    static inline T get(CDT::V2d<T> const& p)
    {
        return p.x;
    }
    static inline void set(CDT::V2d<T>& p, T const& value)
    {
        p.x = value;
    }
};
// Y
template <typename T>
struct access<CDT::V2d<T>, 1>
{
    static inline T get(CDT::V2d<T> const& p)
    {
        return p.y;
    }
    static inline void set(CDT::V2d<T>& p, T const& value)
    {
        p.y = value;
    }
};

} // namespace traits
} // namespace geometry
} // namespace boost

namespace CDT
{

template <typename T>
class PointRTree
{
public:
    void addPoint(const V2d<T>& pos, const VertInd iV)
    {
        m_rtree.insert(std::make_pair(pos, iV));
    }

    VertInd nearestPoint(const V2d<T>& pos) const
    {
        std::vector<VertexPos> query;
        namespace bgi = boost::geometry::index;
        m_rtree.query(bgi::nearest(pos, 1), std::back_inserter(query));
        assert(query.size());
        return query.front().second;
    }

private:
    typedef std::pair<V2d<T>, VertInd> VertexPos;
    boost::geometry::index::
        rtree<VertexPos, boost::geometry::index::linear<16, 4> >
            m_rtree;
};

} // namespace CDT

#endif
