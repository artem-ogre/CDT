cmake_minimum_required(VERSION 3.3...4.0)

project(CDT_visualizer VERSION 1.0.0 LANGUAGES CXX)

# Use Conan toolchain if it exists
set(cdt_conan_toolchain "${CMAKE_CURRENT_BINARY_DIR}/conan_toolchain.cmake")
if(NOT CDT_CONAN_TOOLCHAIN_INCLUDED AND EXISTS ${cdt_conan_toolchain})
    include(${cdt_conan_toolchain})
    set(CDT_CONAN_TOOLCHAIN_INCLUDED ON)
endif()

set(CMAKE_AUTOMOC ON)

find_package(Qt6 CONFIG COMPONENTS Core Widgets Gui REQUIRED)

option(CDT_USE_FIND_PACKAGE
    "If enabled prebuild CDT is consumed with find_package" OFF)

if(CDT_USE_FIND_PACKAGE)
    # add CDT as package
    find_package(CDT REQUIRED CONFIG)
else()
    set(CDT_ENABLE_TESTING OFF)
    # add CDT as source (easier development)
    add_subdirectory(../CDT CDT)
endif()

add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PRIVATE main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE Qt6::Widgets)
target_link_libraries(${PROJECT_NAME} PRIVATE CDT::CDT)
