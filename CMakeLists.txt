cmake_minimum_required(VERSION 2.8.3)
project(uavnav)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    mavros_msgs
    geometry_msgs
    )
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package (Eigen3 REQUIRED)
find_package (yaml-cpp REQUIRED)
find_package (Sophus REQUIRED )


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
    CATKIN_DEPENDS roscpp std_msgs
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    )

######################################################################
add_executable(straight_trj
    src/straight_trj.cpp
    src/movement/generalmove.cpp
    )
target_link_libraries(straight_trj
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${Sophus_LIBRARIES}
    )

######################################################################
add_executable(emsd_demo
    src/emsd_demo.cpp
    src/movement/generalmove.cpp
    src/movement/circletrj.cpp
    )
target_link_libraries(emsd_demo
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${Sophus_LIBRARIES}
    )

######################################################################
add_executable(test_trj_fj005
    src/test_trj_fj005.cpp
    src/movement/generalmove.cpp
    src/movement/circletrj.cpp
    )
target_link_libraries(test_trj_fj005
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${Sophus_LIBRARIES}
    )

######################################################################
add_executable(test_trj_8
    src/test_trj_8.cpp
    src/movement/generalmove.cpp
    src/movement/circletrj.cpp
    )
target_link_libraries(test_trj_8
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${Sophus_LIBRARIES}
    )


