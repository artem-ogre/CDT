cmake_minimum_required(VERSION 3.4)

project(
    "CDT"
    VERSION 1.1.2
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
    "If enabled Boost is used as a fall-back for features missing in C++98
    and performance tweaks (e.g., using boost::flat_set)"
    OFF)

option(CDT_USE_AS_COMPILED_LIBRARY
    "If enabled templates for float and double will be instantiated and compiled into a library")

option(CDT_USE_64_BIT_INDEX_TYPE
    "If enabled 64bits are used to store vertex/triangle index types. Otherwise 32bits are used (up to 4.2bn items)"
    OFF)

option(CDT_ENABLE_TESTING
    "If enabled tests target will ge generated)"
    OFF)

# check if Boost is needed
if(cxx_std_11 IN_LIST CMAKE_CXX_COMPILE_FEATURES)
    # Work-around as AppleClang 11 defaults to c++98 by default
    if(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
        message(STATUS "AppleClang work-around: set CMAKE_CXX_STANDARD to 11")
        set(CMAKE_CXX_STANDARD 11)
    endif()
else()
    message(STATUS "Compiler does not support C++11: falling back to Boost")
    set(CDT_USE_BOOST ON)
endif()

message(STATUS "CDT_USE_BOOST is ${CDT_USE_BOOST}")
message(STATUS "CDT_USE_AS_COMPILED_LIBRARY is ${CDT_USE_AS_COMPILED_LIBRARY}")
message(STATUS "CDT_USE_64_BIT_INDEX_TYPE is ${CDT_USE_64_BIT_INDEX_TYPE}")

# Use boost for c++98 versions of c++11 containers or for Boost::rtree
if(CDT_USE_BOOST)
    find_package(Boost REQUIRED)
endif()


# configure target
set(cdt_include_dirs
    include
    extras
)

if(CDT_USE_AS_COMPILED_LIBRARY)
    set(cdt_scope PUBLIC)
    set(cdt_sources
        src/CDT.cpp
    )
    set(cdt_headers
        include/CDT.h
        include/CDTUtils.h
        include/KDTree.h
        include/LocatorKDTree.h
        include/remove_at.hpp
        include/CDT.hpp
        include/CDTUtils.hpp
        include/predicates.h
        extras/VerifyTopology.h
        extras/InitializeWithGrid.h
    )
    add_library(${PROJECT_NAME} ${cdt_sources} ${cdt_headers})
    # Set symbols visibility to hidden by default
    set_target_properties(
        ${PROJECT_NAME}
        PROPERTIES
            VERSION ${PROJECT_VERSION}
            SOVERSION ${PROJECT_VERSION}
            CXX_VISIBILITY_PRESET hidden
            VISIBILITY_INLINES_HIDDEN 1
    )
    # Generate export header and add it to include directories
    include(GenerateExportHeader)
    generate_export_header(${PROJECT_NAME})
    target_include_directories(
        ${PROJECT_NAME} ${cdt_scope}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    )
    target_sources(
        ${PROJECT_NAME} ${cdt_scope}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/cdt_export.h>
    )

else()
    set(cdt_scope INTERFACE)
    add_library(${PROJECT_NAME} INTERFACE)
endif()

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

foreach(dir_ ${cdt_include_dirs})
    target_include_directories(
        ${PROJECT_NAME}
        ${cdt_scope}
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/${dir_}>
    )
endforeach()

target_include_directories(
    ${PROJECT_NAME} INTERFACE $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_compile_definitions(
    ${PROJECT_NAME}
    ${cdt_scope}
    $<$<BOOL:${CDT_USE_BOOST}>:CDT_USE_BOOST>
    $<$<BOOL:${CDT_USE_AS_COMPILED_LIBRARY}>:CDT_USE_AS_COMPILED_LIBRARY>
    $<$<BOOL:${CDT_USE_64_BIT_INDEX_TYPE}>:CDT_USE_64_BIT_INDEX_TYPE>
)

if(CDT_USE_BOOST)
    target_link_libraries(${PROJECT_NAME} INTERFACE Boost::boost)
endif()


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

install(
  FILES
      $<$<BOOL:${CDT_USE_AS_COMPILED_LIBRARY}>:${CMAKE_CURRENT_BINARY_DIR}/cdt_export.h>
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT ${PROJECT_NAME}Config NAMESPACE CDT:: DESTINATION cmake)

# ------------
# documentation
# ------------

find_package(Doxygen)
if(DOXYGEN_FOUND)
    message(STATUS "Doxygen found: adding documentation target.")
    set(DOXYGEN_EXCLUDE_PATTERNS
        */predicates.h
        conanfile.py
    )
    # doxygen settings can be set here, prefixed with "DOXYGEN_"
    set(DOXYGEN_SOURCE_BROWSER YES)
    set(DOXYGEN_JAVADOC_AUTOBRIEF YES)
    set(DOXYGEN_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/docs")
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE ${PROJECT_SOURCE_DIR}/../docs/README.md)
    set(DOXYGEN_IMAGE_PATH ${PROJECT_SOURCE_DIR}/../docs/images)
    # this target will only be built if specifically asked to.
    # run "make api-docs" to create the doxygen documentation
    doxygen_add_docs(
        ${PROJECT_NAME}-docs
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/../docs
        COMMENT "Generate doxygen documentation for CDT."
    )
    # copy README.md images to doxygen so that they are displayed
    add_custom_command(
        TARGET ${PROJECT_NAME}-docs POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${PROJECT_SOURCE_DIR}/../docs/images
                ${DOXYGEN_OUTPUT_DIRECTORY}/html/images
    )
endif(DOXYGEN_FOUND)


# ------------
# tests
# ------------

if(CDT_ENABLE_TESTING)
    include(CTest)
    enable_testing()
    # add Catch2
    set(Catch2_directory ${PROJECT_SOURCE_DIR}/tests/Catch2)
    add_subdirectory(${Catch2_directory})
    set(CMAKE_MODULE_PATH "${Catch2_directory}/extras" ${CMAKE_MODULE_PATH})
    set(TEST_TARGET_NAME ${PROJECT_NAME}_tests)
    add_executable(${TEST_TARGET_NAME})
    set_property(TARGET ${TEST_TARGET_NAME} PROPERTY CXX_STANDARD 17)
    target_sources(${TEST_TARGET_NAME} PRIVATE tests/cdt.test.cpp)
    target_link_libraries(
        ${TEST_TARGET_NAME}
        PRIVATE
            Catch2::Catch2WithMain
            CDT::CDT
    )
    include(Catch)
    catch_discover_tests(${TEST_TARGET_NAME} WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/tests")
endif()
