/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/**
 * @file
 * Adapter between for boost::geometry::rtree and CDT
 */

#ifndef CDT_o2ROLeC4NKchgja4Awtg
#define CDT_o2ROLeC4NKchgja4Awtg

#include "CDTUtils.h"

#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace CDT
{

/// R-tree holding points
template <typename T>
class LocatorBoostRTree
{
public:
    /// Add point to R-tree
    void addPoint(const VertInd iPoint, const std::vector<V2d<T> >& points)
    {
        const V2d<T>& pos = points[iPoint];
        m_rtree.insert(std::make_pair(Point(pos.x, pos.y), iPoint));
    }
    /// Find nearest point using R-tree
    VertInd nearPoint(const V2d<T>& pos, const std::vector<V2d<T> >&) const
    {
        std::vector<Value> query;
        namespace bgi = boost::geometry::index;
        m_rtree.query(
            bgi::nearest(Point(pos.x, pos.y), 1), std::back_inserter(query));
        assert(query.size());
        return query.front().second;
    }

private:
    typedef boost::geometry::model::point<T, 2, boost::geometry::cs::cartesian>
        Point;
    typedef std::pair<Point, VertInd> Value;
    boost::geometry::index::rtree<Value, boost::geometry::index::linear<16, 4> >
        m_rtree;
};

} // namespace CDT

#endif
