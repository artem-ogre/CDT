/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/**
 * @file
 * Helper function to initialize triangulation with regular grid instead of
 * super-triangle
 */

#ifndef CDT_HmXGv083vZCrT3OXASD9
#define CDT_HmXGv083vZCrT3OXASD9

#include "CDT.h"
#include "CDTUtils.h"

#include <cstddef>
#include <iterator>
#include <vector>

namespace CDT
{
namespace detail
{

/**
 * Generate grid vertices given of X- and Y-ticks
 *
 * @tparam OutputIt output iterator
 * @tparam TXCoordIter iterator dereferencing to X coordinate
 * @tparam TYCoordIter iterator dereferencing to Y coordinate
 * @param outFirst the beginning of the destination range
 * @param xfirst beginning of X-ticks range
 * @param xlast end of X-ticks range
 * @param yfirst beginning of Y-ticks range
 * @param ylast end of Y-ticks range
 */
template <typename OutputIt, typename TXCoordIter, typename TYCoordIter>
void generateGridVertices(
    OutputIt outFirst,
    const TXCoordIter xfirst,
    const TXCoordIter xlast,
    const TYCoordIter yfirst,
    const TYCoordIter ylast)
{
    typedef typename std::iterator_traits<TXCoordIter>::value_type T;
    const std::size_t xres = std::distance(xfirst, xlast) - 1;
    const std::size_t yres = std::distance(yfirst, ylast) - 1;

    TXCoordIter yiter = yfirst;
    for(std::size_t iy = 0; yiter != ylast; ++yiter, ++iy)
    {
        TXCoordIter xiter = xfirst;
        for(std::size_t ix = 0; xiter != xlast; ++xiter, ++ix)
        {
            Vertex<T> v;
            v.pos = V2d<T>::make(*xiter, *yiter);

            const std::size_t i = iy * xres + ix;
            // left-up
            if(ix > 0 && iy < yres)
            {
                v.triangles.push_back(2 * (i - 1));
                v.triangles.push_back(2 * (i - 1) + 1);
            }
            // right-up
            if(ix < xres && iy < yres)
            {
                v.triangles.push_back(2 * i);
            }
            // left-down
            if(ix > 0 && iy > 0)
            {
                v.triangles.push_back(2 * (i - xres - 1) + 1);
            }
            // right-down
            if(ix < xres && iy > 0)
            {
                v.triangles.push_back(2 * (i - xres));
                v.triangles.push_back(2 * (i - xres) + 1);
            }
            *outFirst++ = v;
        }
    }
}

/**
 * Generate grid triangles
 *
 * @tparam OutputIt output iterator
 * @param outFirst the beginning of the destination range
 * @param xres grid X-resolution
 * @param yres grid Y-resolution
 */
template <typename OutputIt>
void generateGridTriangles(
    OutputIt outFirst,
    const std::size_t xres,
    const std::size_t yres)
{
    for(std::size_t iy = 0; iy < yres; ++iy)
    {
        for(std::size_t ix = 0; ix < xres; ++ix)
        {
            // 2___3           v3
            // |\  |           /\
            // | \ |        n3/  \n2
            // |__\|         /____\
            // 0   1       v1  n1  v2
            const std::size_t i = iy * xres + ix;
            const std::size_t iv = iy * (xres + 1) + ix;
            const VertInd vv[4] = {iv, iv + 1, iv + xres + 1, iv + xres + 2};
            Triangle t;

            t.vertices = {vv[0], vv[1], vv[2]};
            t.neighbors = {
                iy ? 2 * i - xres * 2 + 1 : noNeighbor,
                2 * i + 1,
                ix ? 2 * i - 1 : noNeighbor};
            *outFirst++ = t;

            t.vertices = {vv[1], vv[3], vv[2]};
            t.neighbors = {
                ix < xres - 1 ? 2 * i + 2 : noNeighbor,
                iy < yres - 1 ? 2 * i + xres * 2 : noNeighbor,
                2 * i};
            *outFirst++ = t;
        }
    }
}

} // namespace detail

/**
 * Make a triangulation that uses regular grid triangles instead of
 * super-triangle
 *
 * @tparam T type of vertex coordinates (e.g., float, double)
 * @param xmin minimum X-coordinate of grid
 * @param xmax maximum X-coordinate of grid
 * @param ymin minimum Y-coordinate of grid
 * @param ymax maximum Y-coordinate of grid
 * @param xres grid X-resolution
 * @param yres grid Y-resolution
 * @param out triangulation to initialize with grid super-geometry
 */
template <typename T>
void initializeWithRegularGrid(
    const T xmin,
    const T xmax,
    const T ymin,
    const T ymax,
    const std::size_t xres,
    const std::size_t yres,
    Triangulation<T>& out)
{
    std::vector<T> xcoords;
    std::vector<T> ycoords;
    xcoords.reserve(xres + 1);
    ycoords.reserve(yres + 1);
    const T xstep = (xmax - xmin) / xres;
    T x = xmin;
    for(std::size_t ix = 0; ix <= xres; ++ix, x += xstep)
        xcoords.push_back(x);
    const T ystep = (ymax - ymin) / yres;
    T y = ymin;
    for(std::size_t iy = 0; iy <= yres; ++iy, y += ystep)
        ycoords.push_back(y);

    return initializeWithIrregularGrid(
        xcoords.begin(), xcoords.end(), ycoords.begin(), ycoords.end(), out);
}

/**
 * Make a triangulation that uses irregular grid triangles instead of
 * super-triangle. Irregular grid is given by collections of X- and Y-ticks
 *
 * @tparam T type of vertex coordinates (e.g., float, double)
 * @tparam TXCoordIter iterator dereferencing to X coordinate
 * @tparam TYCoordIter iterator dereferencing to Y coordinate
 * @param xfirst beginning of X-ticks range
 * @param xlast end of X-ticks range
 * @param yfirst beginning of Y-ticks range
 * @param ylast end of Y-ticks range
 * @param out triangulation to initialize with grid super-geometry
 */
template <typename T, typename TXCoordIter, typename TYCoordIter>
void initializeWithIrregularGrid(
    const TXCoordIter xfirst,
    const TXCoordIter xlast,
    const TYCoordIter yfirst,
    const TYCoordIter ylast,
    Triangulation<T>& out)
{
    const std::size_t xres = std::distance(xfirst, xlast) - 1;
    const std::size_t yres = std::distance(yfirst, ylast) - 1;
    out.triangles.reserve(xres * yres * 2);
    out.vertices.reserve((xres + 1) * (yres + 1));
    detail::generateGridVertices(
        std::back_inserter(out.vertices), xfirst, xlast, yfirst, ylast);
    detail::generateGridTriangles(
        std::back_inserter(out.triangles), xres, yres);
    out.initializedWithCustomSuperGeometry();
}

} // namespace CDT

#endif
