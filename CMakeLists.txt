cmake_minimum_required(VERSION 2.8.3)
project(uavnav)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

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
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_msgs
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
    )

add_executable(offb_node_infoday_hk
    src/infoday.cpp
    src/writing.cpp)
add_executable(circle
    src/circle.cpp
    src/generalmove.cpp
    src/generalmove.h)
add_executable(rectangle
    src/rectangle.cpp
    src/generalmove.cpp
    src/generalmove.h)
add_executable(step
    src/step.cpp)
add_executable(rotate
    src/rotate.cpp
    src/generalmove.cpp
    src/generalmove.h)
add_executable(circle_w_rotate
    src/circle_w_roatate.cpp
    src/generalmove.cpp
    src/generalmove.h)
add_executable(straight_path
    src/straight_path.cpp
    src/generalmove.cpp
    src/generalmove.h)


target_link_libraries(rectangle
    ${catkin_LIBRARIES}
    )
target_link_libraries(circle
    ${catkin_LIBRARIES}
    )
target_link_libraries(offb_node_infoday_hk
    ${catkin_LIBRARIES}
    )
target_link_libraries(step
    ${catkin_LIBRARIES}
    )
target_link_libraries(rotate
    ${catkin_LIBRARIES}
    )
target_link_libraries(circle_w_rotate
    ${catkin_LIBRARIES}
    )
target_link_libraries(straight_path
    ${catkin_LIBRARIES}
    )
