cmake_minimum_required(VERSION 3.15)
project(vnav_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
    add_compile_options(/W3 /O2)
else()
    add_compile_options(-Wall -Wextra -O2)
endif()

include_directories(${CMAKE_SOURCE_DIR}/include)

set(VNAV_SOURCES
    src/arinc424.cpp
    src/lnav.cpp
    src/vnav.cpp
    src/trajectory_solver.cpp
)

set(VNAV_HEADERS
    include/vnav/isa.h
    include/vnav/great_circle.h
    include/vnav/arinc424.h
    include/vnav/lnav.h
    include/vnav/vnav.h
    include/vnav/trajectory_solver.h
)

add_library(vnav STATIC ${VNAV_SOURCES} ${VNAV_HEADERS})
target_include_directories(vnav PUBLIC ${CMAKE_SOURCE_DIR}/include)

find_package(Python3 COMPONENTS Interpreter Development)
if(Python3_FOUND)
    message(STATUS "Python3 found: ${Python3_EXECUTABLE} (${Python3_VERSION})")

    execute_process(
        COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
        OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE PYBIND11_RESULT
    )

    if(PYBIND11_RESULT EQUAL 0)
        message(STATUS "pybind11 cmake dir: ${PYBIND11_CMAKE_DIR}")
        set(pybind11_DIR ${PYBIND11_CMAKE_DIR})
        find_package(pybind11 CONFIG)
    endif()

    if(pybind11_FOUND)
        message(STATUS "pybind11 found")
        pybind11_add_module(vnav_core python/bindings.cpp)
        target_link_libraries(vnav_core PRIVATE vnav)
        target_include_directories(vnav_core PRIVATE ${CMAKE_SOURCE_DIR}/include)
    else()
        message(STATUS "pybind11 not found, Python bindings will not be built")
    endif()
else()
    message(STATUS "Python3 not found, Python bindings will not be built")
endif()

option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
