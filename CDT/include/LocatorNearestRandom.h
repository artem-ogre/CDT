/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/**
 * @file
 * Locator that returns nearest random point
 */

#ifndef CDT_qG9bX9lZ2nG4uW8xF5kC
#define CDT_qG9bX9lZ2nG4uW8xF5kC

#include "CDTUtils.h"

#include <limits>
#include <utility>
#include <vector>

namespace CDT
{

/// Locator that returns nearest random point
template <typename T>
class LocatorNearestRandom
{
public:
    LocatorNearestRandom()
        : m_nSamples(10)
        , m_randGen(m_randGenSeed)
    {}
    LocatorNearestRandom(VertInd nSamples)
        : m_nSamples(nSamples)
        , m_randGen(m_randGenSeed)
    {}
    /// Add point to R-tree
    void addPoint(const VertInd iPoint, const std::vector<V2d<T> >&)
    {
        m_vertices.push_back(iPoint);
    }
    /// Find nearest random point
    VertInd
    nearPoint(const V2d<T>& pos, const std::vector<V2d<T> >& points) const
    {
        VertInd nearest = 0;
        T minDistSq = std::numeric_limits<T>::max();
        for(std::size_t iSample = 0; iSample < m_nSamples; ++iSample)
        {
            const VertInd iRand(m_randGen() % m_vertices.size());
            const VertInd iV = m_vertices[iRand];
            const T distSq = distanceSquared(points[iV], pos);
            if(distSq < minDistSq)
            {
                minDistSq = distSq;
                nearest = iV;
            }
        }
        return nearest;
    }

private:
    std::vector<VertInd> m_vertices;
    VertInd m_nSamples;
    static const unsigned int m_randGenSeed = 9001;
    mutable mt19937 m_randGen;
};

} // namespace CDT

#endif
