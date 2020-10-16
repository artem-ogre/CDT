cmake_minimum_required(VERSION 3.3)

project(CDT_visualizer VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)

find_package(Qt5 COMPONENTS Core Widgets Gui REQUIRED)
find_package(CDT REQUIRED)

add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PRIVATE main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE Qt5::Widgets)
target_link_libraries(${PROJECT_NAME} PRIVATE CDT::CDT)