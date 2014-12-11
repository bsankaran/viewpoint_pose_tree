cmake_minimum_required(VERSION 2.8.3)
project(ann_catkin)
#project(ANN)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

set(PKGVERSION "ann_1.1.1")
set(PATH_ANN_LIBRARY ${PROJECT_SOURCE_DIR}/${PKGVERSION}/lib)

# Bharath's ugly working solution, that he's not really proud of
execute_process(
  COMMAND mkdir -p lib
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/${PKGVERSION}
  )
# Daniel's ugly but working solution! Thanks Daniel! He is really proud of it
execute_process(
  COMMAND make targets "ANNLIB = libANN.a" "C++ = g++" "CFLAGS = -fPIC -O3" "MAKELIB = ar ruv" "RANLIB = true"
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/${PKGVERSION}/src
  )

find_library(ann_catkin NAMES ANN PATHS "${PATH_ANN_LIBRARY}")
message(STATUS "${ann_catkin}")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also nee
catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/${PKGVERSION}/include
  LIBRARIES ${ann_catkin}
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

install(FILES ${ann_catkin}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
