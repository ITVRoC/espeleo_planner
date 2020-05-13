# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

project( recon_surface )


cmake_minimum_required(VERSION 2.8.11)

# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )
find_package(Eigen3 REQUIRED)
find_package(PNG REQUIRED)


if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()

# include helper file
include( ${CGAL_USE_FILE} )

# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()

# include for local directory

# include for local package

include_directories(${EIGEN3_INCLUDE_DIR} ${PNG_INCLUDE_DIR} )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++14 -g")

# Creating entries for target: recon_surface
# ############################

file(GLOB recon_cpps src/*.cpp )
add_executable( recon_surface ${recon_cpps})

add_to_cached_list( CGAL_EXECUTABLE_TARGETS recon_surface )

# Link the executable to CGAL and third-party libraries
target_link_libraries(recon_surface   ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${PNG_LIBRARY})

