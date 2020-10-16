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
option(CDT_USE_BOOST
    "If enabled Boost::rtree can be used for finding the nearest point; boost::flat_set is used for better performance"
    OFF)

option(CDT_USE_AS_COMPILED_LIBRARY
    "If enabled templates for float and double will be instantiated and compiled into a library")

# check if Boost is needed
if(cxx_std_11 IN_LIST CMAKE_CXX_COMPILE_FEATURES)
    # options
    option(CDT_USE_BOOST
        "If enabled Boost::rtree can be used for finding the nearest point; boost::flat_set is used for better performance"
        OFF)
    message(STATUS "CDT_USE_BOOST is ${CDT_USE_BOOST}")
else()
    message(STATUS "Compiler does not support C++11: falling back to Boost")
    set(CDT_USE_BOOST ON)
endif()

message(STATUS "CDT_USE_AS_COMPILED_LIBRARY is ${CDT_USE_AS_COMPILED_LIBRARY}")

# Use boost for c++98 versions of c++11 containers or for Boost::rtree
if(CDT_USE_BOOST)
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

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

foreach(dir_ ${cdt_include_dirs})
    target_include_directories(
        ${PROJECT_NAME} ${cdt_scope} $<BUILD_INTERFACE:${dir_}>)
endforeach()
target_include_directories(
    ${PROJECT_NAME} INTERFACE $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_compile_definitions(
    ${PROJECT_NAME}
    ${cdt_scope}
    $<$<BOOL:${CDT_USE_BOOST}>:CDT_USE_BOOST>
    $<$<BOOL:${CDT_USE_AS_COMPILED_LIBRARY}>:CDT_USE_AS_COMPILED_LIBRARY>
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
