/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#ifndef CDT_o2ROLeC4NKchgja4Awtg
#define CDT_o2ROLeC4NKchgja4Awtg

#include "CDTUtils.h"

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

// Specialize boost::geometry point traits for CDT's 2D vector type
namespace boost
{
namespace geometry
{
namespace traits
{

// point traits
/// Point tag
template <typename T>
struct tag<CDT::V2d<T> >
{
    typedef point_tag type; ///< point tag
};

/// Dimension (2d)
template <typename T>
struct dimension<CDT::V2d<T> > : boost::mpl::int_<2>
{};

/// Coordinate type
template <typename T>
struct coordinate_type<CDT::V2d<T> >
{
    typedef T type; ///< type
};

/// Cartesian coordinate system
template <typename T>
struct coordinate_system<CDT::V2d<T> >
{
    typedef cs::cartesian type; ///< cartesian
};

// point access
/// X
template <typename T>
struct access<CDT::V2d<T>, 0>
{
    /// X-getter
    static inline T get(CDT::V2d<T> const& p)
    {
        return p.x;
    }
    /// X-setter
    static inline void set(CDT::V2d<T>& p, T const& value)
    {
        p.x = value;
    }
};
/// Y
template <typename T>
struct access<CDT::V2d<T>, 1>
{
    /// Y-getter
    static inline T get(CDT::V2d<T> const& p)
    {
        return p.y;
    }
    /// Y-setter
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

/// R-tree holding points
template <typename T>
class PointRTree
{
public:
    /// Add point to R-tree
    void addPoint(const V2d<T>& pos, const VertInd iV)
    {
        m_rtree.insert(std::make_pair(pos, iV));
    }
    /// Find nearest point using R-tree
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
