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

namespace CDT
{
namespace detail
{

/**
 * Generate grid vertices
 *
 * @tparam T type of vertex coordinates (e.g., float, double)
 * @tparam OutputIt output iterator
 * @param outFirst the beginning of the destination range
 * @param xmin minimum X-coordinate of grid
 * @param xmax maximum X-coordinate of grid
 * @param ymin minimum Y-coordinate of grid
 * @param ymax maximum Y-coordinate of grid
 * @param xres grid X-resolution
 * @param yres grid Y-resolution
 */
template <typename T, typename OutputIt>
void generateGridVertices(
    OutputIt outFirst,
    const T xmin,
    const T xmax,
    const T ymin,
    const T ymax,
    const std::size_t xres,
    const std::size_t yres)
{
    const T xstep = (xmax - xmin) / xres;
    const T ystep = (ymax - ymin) / yres;
    T y = ymin;
    for(std::size_t iy = 0; iy <= yres; ++iy, y += ystep)
    {
        T x = xmin;
        for(std::size_t ix = 0; ix <= xres; ++ix, x += xstep)
        {
            Vertex<T> v;
            v.pos = V2d<T>::make(x, y);

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
void initializeWithGrid(
    const T xmin,
    const T xmax,
    const T ymin,
    const T ymax,
    const std::size_t xres,
    const std::size_t yres,
    Triangulation<T>& out)
{
    out.triangles.reserve(xres * yres * 2);
    out.vertices.reserve((xres + 1) * (yres + 1));
    detail::generateGridVertices(
        std::back_inserter(out.vertices), xmin, xmax, ymin, ymax, xres, yres);
    detail::generateGridTriangles(
        std::back_inserter(out.triangles), xres, yres);
    out.initializedWithCustomSuperGeometry();
}

} // namespace CDT

#endif
