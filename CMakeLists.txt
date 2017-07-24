cmake_minimum_required(VERSION 2.8.3)
project(mesh_to_traversability_gridmap)
set(CMAKE_BUILD_TYPE RELEASE)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_package(INCLUDE_DIRS include)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})


cs_add_library(mesh_to_traversability_gridmap
  src/library/mesh_to_traversability_gridmap.cpp
)

# create node to connect to voxblox/mesh and compute traversability  gridMap 
cs_add_executable(mesh_to_traversability_gridmap_node
  src/mesh_to_traversability_gridmap_node.cpp
  include/mesh_to_traversability_gridmap/mesh_to_traversability_gridmap.hpp
)
target_link_libraries(mesh_to_traversability_gridmap_node mesh_to_traversability_gridmap ${catkin_LIBRARIES})

#install(TARGETS mesh_to_grid_map_save_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
cs_install()
cs_export()
