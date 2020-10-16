cmake_minimum_required(VERSION 3.4)

project(
    "CDT"
    VERSION 0.0.1
    DESCRIPTION
    "C++ library for constrained Delaunay triangulation (CDT)"
    HOMEPAGE_URL "https://github.com/artem-ogre/CDT"
    LANGUAGES CXX)


# -------
# target
# -------


# export all symbols as we do it by instantiating templates anyway
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)


# options
option(CDT_DONT_USE_BOOST_RTREE
    "If enabled Boost::rtree is used for finding the nearest point when adding new points"
    ON)

option(CDT_USE_AS_COMPILED_LIBRARY
    "If enabled templates for float and double will be instantiated and compiled into a library")

message(STATUS "CDT_DONT_USE_BOOST_RTREE is ${CDT_DONT_USE_BOOST_RTREE}")
message(STATUS "CDT_USE_AS_COMPILED_LIBRARY is ${CDT_USE_AS_COMPILED_LIBRARY}")


# check if Boost is needed
if(NOT cxx_std_11 IN_LIST CMAKE_CXX_COMPILE_FEATURES)
    set(CDT_NO_CXX11 ON)
    message(STATUS "Compiler does not support C++11: falling back to Boost")
endif()

# Use boost for c++98 versions of c++11 containers or for Boost::rtree
if(CDT_NO_CXX11 OR (NOT CDT_DONT_USE_BOOST_RTREE))
    find_package(Boost REQUIRED)
endif()


# configure target
set(cdt_include_dirs
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/extras
)

if(CDT_USE_AS_COMPILED_LIBRARY)
    set(cdt_scope PUBLIC)
    set(cdt_sources src/CDT.cpp)
    add_library(${PROJECT_NAME} ${cdt_sources})
else()
    set(cdt_scope INTERFACE)
    add_library(${PROJECT_NAME} INTERFACE)
endif()

foreach(dir_ ${cdt_include_dirs})
    target_include_directories(
        ${PROJECT_NAME} ${cdt_scope} $<BUILD_INTERFACE:${dir_}>)
endforeach()
target_include_directories(
    ${PROJECT_NAME} INTERFACE $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_compile_definitions(
    ${PROJECT_NAME}
    ${cdt_scope}
    $<$<BOOL:CDT_DONT_USE_BOOST_RTREE>:CDT_DONT_USE_BOOST_RTREE>
    $<$<BOOL:CDT_USE_AS_COMPILED_LIBRARY>:CDT_USE_AS_COMPILED_LIBRARY>
)


# -------------
# installation
# -------------


include(GNUInstallDirs)

# install and export the library
foreach(dir_ ${cdt_include_dirs})
    install(DIRECTORY ${dir_}/ DESTINATION include)
endforeach()

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Config
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(EXPORT ${PROJECT_NAME}Config NAMESPACE CDT:: DESTINATION cmake)
