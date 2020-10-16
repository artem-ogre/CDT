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

if(NOT cxx_std_11 IN_LIST CMAKE_CXX_COMPILE_FEATURES)
    # use boost for c++98 versions of c++11 containers as unordered_map
    find_package(Boost REQUIRED)
endif()

set(cdt_include_dirs
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/extras
)

if(CDT_USE_AS_COMPILED_LIBRARY)
    set(cdt_sources src/CDT.cpp)
    add_library(${PROJECT_NAME} ${cdt_sources})
    foreach(dir_ ${cdt_include_dirs})
        target_include_directories(
            ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${dir_}>)
    endforeach()
    target_compile_definitions(
        ${PROJECT_NAME} PUBLIC CDT_USE_AS_COMPILED_LIBRARY)
else()
    add_library(${PROJECT_NAME} INTERFACE)
    foreach(dir_ ${cdt_include_dirs})
        target_include_directories(
            ${PROJECT_NAME} INTERFACE $<BUILD_INTERFACE:${dir_}>)
    endforeach()
endif()

target_include_directories(
    ${PROJECT_NAME} INTERFACE $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

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
