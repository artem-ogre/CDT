<img src="./images/CDT_logo.png" alt="CDT Logo" height="120"/>

[![CI Builds](https://github.com/artem-ogre/CDT/workflows/CI%20Builds/badge.svg)](https://github.com/artem-ogre/CDT/actions/)

***If CDT helped you please consider adding a star on GitHub. This means a lot to the authors*** 🤩

# CDT: Conforming or Constrained Delaunay Triangulation
## What is CDT?
CDT is a C++ library for generating constraint or conforming Delaunay triangulations.
- **open-source:** permissively-licensed under Mozilla Public License (MPL) 2.0
- **cross-platform:** tested on Windows, Linux (Ubuntu), and macOS
- **portable:** backwards-compatible with C++98
- **bloat-free:** no external dependencies by default
- **flexible:** can be consumed as a header-only or as a compiled library
- **performant:** continuously profiled, measured, and optimized
- **numerically robust:** triantulation algorithms rely on robust geometric predicates

## What can CDT do?
<img src="./images/show-case.png" alt="CDT Logo" height="150"/>

- Constrained Delaunay Triangulations: force edges into Delaunay triangulation
- Conforming Delaunay Triangulations: add new points into Delaunay triangulation until the edge is present in triangulation
- Convex-hulls
- Automatically finding and removing holes

## What corner-cases can CDT handle?
<img src="./images/corner-cases.png" alt="CDT Logo" height="180"/>

- Points exactly on the edges
- Exactly overlapping edges
- Resolving intersecting edges by adding points at the intersections (with `CDT::IntersectingConstraintEdges::Resolve`)



**Table of Contents**
- [Online Documentation](#online-doc)
- [Algorithm](#algorithm)
- [Implementation Details](#details)
- [Installation/Building](#installation)
- [Using with Code Examples](#using)
- [Contributors](#contributors)
- [Contributing](#contributing)
- [Example Gallery](#example-gallery)
- [Bibliography](#bibliography)

## <a name="online-doc"/>Online Documentation</a>
[**Latest online documentation**](https://artem-ogre.github.io/CDT/doxygen/index.html) (automatically generated with Doxygen).

## <a name="algorithm"/>Algorithm</a>

- Implementation closely follows incremental construction algorithm by Anglada [[1](#1)]. 
- During the legalization, the cases
when at least one vertex belongs to super-triangle are resolved using an approach as described in Žalik et. al [[2](#2)].
- For finding a triangle that contains inserted point remembering randomized triangle walk is used [[3](#3)]. To find the starting triangle for the walk the nearest point is found using a kd-tree with mid-split nodes.
- By default inserted vertices are randomly shuffled internally to improve performance and avoid worst-case scenarios. The original vertices order can be optied-in using `VertexInsertionOrder::AsProvided` when constructing a triangulation. 

**Pre-conditions:**
- No duplicated points (use provided functions for removing duplicate points and re-mapping edges)
- No two constraint edges intersect each other (overlapping boundaries are allowed)

**Post-conditions:**
- Triangles have counter-clockwise (CCW) winding

## <a name="details"/>Implementation Details</a>

- Supports three ways of removing outer triangles:
    - `eraseSuperTriangle`: produce a convex-hull
    - `eraseOuterTriangles`: remove all outer triangles until a boundary defined by constraint edges
    - `eraseOuterTrianglesAndHoles`: remove outer triangles and automatically detected holes. Starts from super-triangle and traverses triangles until outer boundary. Triangles outside outer boundary will be removed. Then traversal continues until next boundary. Triangles between two boundaries will be kept. Traversal to next boundary continues (this time removing triangles). Stops when all triangles are traversed.
- Supports [overlapping boundaries](#overlapping-boundaries-example)

- Removing duplicate points and re-mapping constraint edges can be done using functions: `RemoveDuplicatesAndRemapEdges, RemoveDuplicates,  RemapEdges`

- Uses William C. Lenthe's implementation of robust orientation and in-circle geometric predicates: https://github.com/wlenthe/GeometricPredicates.

- Boost is an optional dependency used for:
    * **Fall back** for standard library features missing in C++98 compilers.
    * **Minor performance tweaks:** `boost::container::flat_set` is used for faster triangle walking


    To opt in define `CDT_USE_BOOST` either in CMake or in a preprocessor.

- A demonstrator tool is included: requires Qt for GUI. When running demo-tool **make sure** that working directory contains files from 'data' folder.

## <a name="installation"/>Installation/Building</a>

CDT uses modern CMake and should *just work* out of the box without any suprises. The are many ways to consume CDT: 
- copy headers and use as a header-only library
- add to CMake project directly with `add_subdirectory`
- pre-build and add to CMake project as a dependency with `find_package`
- consume as a Conan package

**CMake options**

<table>
<thead>
<tr>
<th>Option</th>
<th>Default value</th>
<th>Description</th>
</tr>
</thead>
<tbody>
<tr>
<td><b>CDT_USE_BOOST</b></td>
<td>OFF</td>
<td>
If enabled Boost is used as a fall-back for features missing in C++98 and performance tweaks (e.g., using boost::flat_set)
</td>
</tr>
<tr>
<td><b>CDT_USE_64_BIT_INDEX_TYPE</b></td>
<td>OFF</td>
<td>
If enabled 64bits are used to store vertex/triangle index types. Otherwise 32bits are used (up to 4.2bn items)
</td>
</tr>
<tr>
<td><b>CDT_USE_AS_COMPILED_LIBRARY</b></td>
<td>OFF</td>
<td>
If enabled templates for float and double will be instantiated and compiled into a library
</td>
</tr>
</tbody>
</table>

**Adding to CMake project directly**

Can be done with [`add_subdirectory`](https://cmake.org/cmake/help/latest/command/add_subdirectory.html) command (e.g., see CDT visualizer's CMakeLists.txt).
```Cmake
# add CDT as subdirectory to CMake project
add_subdirectory(../CDT CDT)
```
**Adding to non-CMake project directly**

To use as **header-only** copy headers from `CDT/include`

To use as a **compiled library** define `CDT_USE_AS_COMPILED_LIBRARY` and compile `CDT.cpp`

**Consume pre-build CDT in CMake project with [`find_package`](https://cmake.org/cmake/help/latest/command/find_package.html)**

CDT provides package config files that can be included by other projects to find and use it.

```bash
# from CDT folder
mkdir build && cd build
# configure with desired CMake flags
cmake -DCDT_USE_AS_COMPILED_LIBRARY=ON -DCDT_USE_BOOST=ON ..
# build and install
cmake --build . && cmake --install .
```

```CMake
# In consuming CMakeLists.txt
find_package(CDT REQUIRED CONFIG)
```

**Consume as [Conan](https://conan.io/) package**

There's a `conanfile.py` recipe provided.
Note that it might need small adjustments like changing boost version to fit your needs.

## <a name="using"/>Using</a>

Public API is provided in two places:
- [`CDT::Triangulation`](https://artem-ogre.github.io/CDT/doxygen/classCDT_1_1Triangulation.html) class is used for performing constrained Delaunay triangulations.
- Free functions in [`CDT.h`](https://artem-ogre.github.io/CDT/doxygen/CDT_8h.html) provide some additional functionality for removing duplicates, re-mapping edges and triangle depth-peeling


### Code Examples

**Delaunay triangulation without constraints (triangulated convex-hull)**

<img src="./images/LakeSuperior.png" alt="Example of a triangulated convex hull" height="150"/>

```c++
#include "CDT.h"
CDT::Triangulation<double> cdt;
cdt.insertVertices(/* points */);
cdt.eraseSuperTriangle();
/* access triangles */ = cdt.triangles;
/* access vertices */ = cdt.vertices;
/* access boundary edges */ = cdt.edges;
```

**Constrained Delaunay triangulation (auto-detected boundaries and holes)**

<img src="./images/A.png" alt="Example of a triangulation with constrained boundaries and auto-detected holes" height="150"/>

```c++
// ... same as above
cdt.insertVertices(/* points */);
cdt.insertEdges(/* boundary edges */);
cdt.eraseOuterTrianglesAndHoles();
/* access triangles */ = cdt.triangles;
/* access vertices */ = cdt.vertices;
/* access boundary edges */ = cdt.edges;
```

**Conforming Delaunay triangulation**

Use `conformToEdges` instead of `insertEdges`

**Resolve edge intersections by adding new points and splitting edges**

Pass `CDT::IntersectingConstraintEdges::Resolve` to `Triangulation` constructor.

**Custom point/edge type**

```c++
struct CustomPoint2D
{
    double data[2];
};

struct CustomEdge
{
    std::pair<std::size_t, std::size_t> vertices;
};

// containers other than std::vector will work too
std::vector<CustomPoint2D> points = /*...*/; 
std::vector<CustomEdge> edges = /*...*/;
CDT::Triangulation<double> cdt;
cdt.insertVertices(
    points.begin(),
    points.end(),
    [](const CustomPoint2D& p){ return p.data[0]; },
    [](const CustomPoint2D& p){ return p.data[1]; }
);
cdt.insertEdges(
    edges.begin(),
    edges.end(),
    [](const CustomEdge& e){ return e.vertices.first; },
    [](const CustomEdge& e){ return e.vertices.second; }
);
```
## <a name="contributors"/>Contributors</a>
- [Artem Amirkhanov](https://github.com/artem-ogre)
- [Karl Åkerblom](https://github.com/kalleakerblom)
- [baiwenlei](https://github.com/baiwenlei): dragging and zooming in the viewer
- [Bärbel Holm](https://github.com/eisbaerli): removing duplicates and re-mapping edges
- [Andre Fecteau](https://github.com/AndreFecteau): benchmarking, profiling, and providing a kd-tree implementation a derivative of which is included in CDT

## <a name="contributing"/>Contributing</a>
Any feedback and contributions are welcome.

## <a name="license"/>License</a>

[Mozilla Public License,  v. 2.0](https://www.mozilla.org/en-US/MPL/2.0/FAQ/)

## <a name="example-gallery"/>Example Gallery</a>
<img src="./images/A.png" alt="A" height="200"/> <img src="./images/Bean.png" alt="Bean" height="200"/> <img src="./images/Guitar.png" alt="Guitar" height="200"/> <img src="./images/Guitar_no_holes.png" alt="Guitar with holes" height="200"/> <img src="./images/LakeSuperior.png" alt="Lake Superior" height="200"/> <img src="./images/Sweden.png" alt="Sweden" height="200"/> <a name="overlapping-boundaries-example"/><img src="./images/Overlapping_boundaries.png" alt="Overlapping boundaries" height="200"/></a> 

## <a name="bibliography"/>Bibliography</a>
<a name="1">[1]</a> Marc Vigo Anglada,
An improved incremental algorithm for constructing restricted Delaunay triangulations,
_Computers & Graphics_,
Volume 21, Issue 2,
1997,
Pages 215-223,
ISSN 0097-8493.

<a name="2">[2]</a> Borut   Žalik  and  Ivana   Kolingerová,
An incremental construction algorithm for Delaunay triangulation using the nearest-point paradigm,
_International Journal of Geographical Information Science_,
Volume 17,
Issue 2,
Pages 119-138,
2003,
DOI 10.1080/713811749.

<a name="3">[3]</a> Olivier Devillers, Sylvvain Pion, Monique Tellaud,
Walking in a triangulation,
_International Journal of Foundations of Computer Science_,
Volume 13,
Issue 2,
Pages 181-199,
2002


