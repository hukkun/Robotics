cmake_minimum_required(VERSION 2.8.3)
project(pcl_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # pcl_conversions
  # roscpp
  # rospy
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  object_msgs
)

find_package(PCL 1.8 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


#_tutorials/CMakeLists.txt:160 (target_link_libraries):
  # Attempt to add link library "/usrcl_tutorials/CMakeLists.txt:160 (target_link_libraries):
  # Attempt to add link library "/usr# Uncomment this if the package has a setup.py. This macro ensures
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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
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
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_tutorials
#  CATKIN_DEPENDS pcl_conversions roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable (voxel_grid src/voxel_grid.cpp)
add_executable (cluster_extraction src/cluster_extraction.cpp)
add_executable (planar_segmentation src/planar_segmentation.cpp)
add_executable (extract_indices src/extract_indices.cpp)
add_executable (normal_estimation_using_integral_images src/normal_estimation_using_integral_images.cpp)
add_executable (cluster_test src/cluster_test.cpp)
add_executable (cluster_velodyne src/cluster_velodyne.cpp)
add_executable (normal_estimation_sq src/normal_estimation_sq.cpp)
add_executable (rm_planar src/rm_planar.cpp)
add_executable (pub_flag src/pub_flag.cpp)
add_executable (point_union src/point_union.cpp)
add_executable (statistical_removal src/statistical_removal.cpp)
add_executable (sq_lidar_filter src/sq_lidar_filter.cpp)
add_executable (save_PointCloud2pcd src/save_PointCloud2pcd.cpp)
add_executable (pub_clusterWithNormal src/pub_clusterWithNormal.cpp)
add_executable (coloring_cluster src/coloring_cluster.cpp)
add_executable (select_cluster src/select_cluster.cpp)
add_executable (centerlaser2odom_pos src/centerlaser2odom_pos.cpp)
add_executable (centerlaser2odom_pt src/centerlaser2odom_pt.cpp)
add_executable (point2csv src/point2csv.cpp)
add_executable (vicon2csv src/vicon2csv.cpp)
add_executable (velodyne2map_pos src/velodyne2map_pos.cpp)
add_executable (velodyne2map_pos2 src/velodyne2map_pos2.cpp)
add_executable (velodyne2map_pt src/velodyne2map_pt.cpp)
add_executable (velodyne2map_pt2 src/velodyne2map_pt2.cpp)
add_executable (pcd_loader src/pcd_loader.cpp)

target_link_libraries (voxel_grid ${PCL_LIBRARIES})
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})
target_link_libraries (planar_segmentation ${PCL_LIBRARIES})
target_link_libraries (extract_indices ${PCL_LIBRARIES})
target_link_libraries (normal_estimation_using_integral_images ${PCL_LIBRARIES})
target_link_libraries (cluster_test ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (cluster_velodyne ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (normal_estimation_sq ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (rm_planar ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (pub_flag ${catkin_LIBRARIES})
target_link_libraries (point_union ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (statistical_removal ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (sq_lidar_filter ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (save_PointCloud2pcd ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (pub_clusterWithNormal ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (coloring_cluster ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (select_cluster ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (centerlaser2odom_pos ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (centerlaser2odom_pt ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (point2csv ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (vicon2csv ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (velodyne2map_pos ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (velodyne2map_pos2 ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (velodyne2map_pt ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (velodyne2map_pt2 ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (pcd_loader ${PCL_LIBRARIES} ${catkin_LIBRARIES})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pcl_tutorials.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/pcl_tutorials_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pcl_tutorials.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
