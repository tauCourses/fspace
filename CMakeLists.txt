project(fspace)
cmake_minimum_required(VERSION 2.8.11)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(CGAL QUIET COMPONENTS)

if (NOT CGAL_FOUND)

    message(STATUS "This project requires the CGAL library, and will not be compiled.")
    return()

endif ()

# include helper file
include(${CGAL_USE_FILE})


# Boost and its components
find_package(Boost REQUIRED)

if (NOT Boost_FOUND)

    message(STATUS "This project requires the Boost library, and will not be compiled.")

    return()

endif ()

add_executable(fspace main.cpp)
add_to_cached_list(CGAL_EXECUTABLE_TARGETS fspace)

target_link_libraries(fspace ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})
