cmake_minimum_required(VERSION 2.8.3)
project(vision_module)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslib
  ann_catkin
  pcl
  util_pkg
  virtual_kinect_pkg
  vocabulary_tree
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS system random)
find_package(VTK REQUIRED)
if (VTK_FOUND)
  message(STATUS "VTK FOUND")
  include_directories(${VTK_INCLUDE_DIRS})
  link_directories(${VTK_LIBRARY_DIRS})
else()
  message(STATUS "ERROR: VTK NOT FOUND")
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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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
  INCLUDE_DIRS include
  LIBRARIES vtree_user
  CATKIN_DEPENDS roslib ann_catkin pcl util_pkg virtual_kinect_pkg vocabulary_tree
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include ${VTK_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(vtree_user src/vtree_user.cpp src/tabletop_analyzer.cpp)

## Declare a cpp executable
# add_executable(vision_module_node src/vision_module_node.cpp)
add_executable(ply2traindata src/ply2traindata.cpp)
add_executable(build_tree src/build_tree.cpp)
add_executable(build_omap src/build_omap.cpp)
add_executable(build_omap_occ src/build_omap_occ.cpp)
add_executable(pcd_from_ply src/pcd_from_ply.cpp)
add_executable(score_pcd src/score_pcd.cpp)
add_executable(compute_confmat src/compute_confmat.cpp)
add_executable(analyze_pcd_with_noise src/analyze_pcd_with_noise.cpp)
add_executable(vpm src/views_to_pcd_model.cpp)

target_link_libraries(vtree_user vtkCommon vtkIO ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(ply2traindata vtkCommon vtkIO ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(build_tree vtree_user ${catkin_LIBRARIES})
target_link_libraries(build_omap vtree_user vtkCommon vtkIO ${catkin_LIBRARIES})
target_link_libraries(build_omap_occ vtree_user vtkCommon vtkIO ${catkin_LIBRARIES})
target_link_libraries(pcd_from_ply vtree_user vtkCommon vtkIO ${catkin_LIBRARIES})
target_link_libraries(score_pcd vtree_user ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(compute_confmat vtree_user vtkCommon vtkIO ${catkin_LIBRARIES})
target_link_libraries(analyze_pcd_with_noise vtree_user ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(vpm ${PCL_LIBRARIES} ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(vision_module_node vision_module_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(vision_module_node
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
# install(TARGETS vision_module vision_module_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vision_module.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
