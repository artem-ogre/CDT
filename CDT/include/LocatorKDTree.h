/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/**
 * @file
 * Adapter between for KDTree and CDT
 */

#ifndef CDT_POINTKDTREE_H
#define CDT_POINTKDTREE_H

#include "CDTUtils.h"
#include "KDTree.h"

namespace CDT
{

/// KD-tree holding points
template <typename T>
class LocatorKDTree
{
public:
    /// Add point to R-tree
    void addPoint(const VertInd i, const std::vector<V2d<T> >& points)
    {
        m_kdTree.insert(i, points);
    }
    /// Find nearest point using R-tree
    VertInd
    nearPoint(const V2d<T>& pos, const std::vector<V2d<T> >& points) const
    {
        return m_kdTree.nearest(pos, points).second;
    }

private:
    KDTree::KDTree<T, 20> m_kdTree;
};

} // namespace CDT

#endif
