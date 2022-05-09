cmake_minimum_required(VERSION 3.0.2)
project(recon_surface)

## Compile as C++11, supported in ROS Kinetic and newer
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++14)
#add_compile_options(-std=c++11 -Og)
#add_definitions(-Wall -Og)
#add_definitions(-Wall -Wno-deprecated -Wno-dev -static -Bstatic -std=gnu++0x)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  visualization_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  std_srvs
  octomap_ros
  octomap_msgs
  dynamic_reconfigure
  nodelet
  pcl_ros
)

find_package(CGAL QUIET COMPONENTS)
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

if(NOT Boost_FOUND)
	message(STATUS "This project requires the Boost library, and will not be compiled.")
	return()
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
	FILES
	MeshFromPointCloud2.srv
	NormalsFromPointCloud2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
	sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
    config/ReconSurface.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mesh_reconstruction
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(recon_surface src/main.cpp src/hole_filling.cpp src/my_png.cpp src/pset.cpp src/rm_artifacts.cpp src/surface_recon.cpp src/texture.cpp)
add_dependencies(recon_surface recon_surface_generate_messages_cpp)
target_link_libraries(recon_surface ${catkin_LIBRARIES} ${LINK_LIBS} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${PNG_LIBRARY})

add_executable(recon_surface_service src/recon_service_from_pointcloud.cpp src/hole_filling.cpp src/my_png.cpp src/pset.cpp src/rm_artifacts.cpp src/surface_recon.cpp src/texture.cpp)
add_dependencies(recon_surface_service recon_surface_generate_messages_cpp)
target_link_libraries(recon_surface_service ${catkin_LIBRARIES} ${LINK_LIBS} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${PNG_LIBRARY})

add_executable(p_normal_service src/p_normal_service_from_pointcloud.cpp src/hole_filling.cpp src/my_png.cpp src/pset.cpp src/rm_artifacts.cpp src/surface_recon.cpp src/texture.cpp)
add_dependencies(p_normal_service recon_surface_generate_messages_cpp)
target_link_libraries(p_normal_service ${catkin_LIBRARIES} ${LINK_LIBS} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${PNG_LIBRARY})
