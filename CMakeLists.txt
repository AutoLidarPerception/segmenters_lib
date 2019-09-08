cmake_minimum_required(VERSION 2.8.3)
project(segmenters_lib)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
#add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    # ROS core
    roscpp
    # ROS messages
    #std_msgs
    #sensor_msgs
    # ROS PCL
    #pcl_conversions
    #pcl_ros
    # 3rd modules for ROS, like Eigen
    cmake_modules
    # local packages
    common_lib
    object_builders_lib
)

## System dependencies are found with CMake's conventions
### PCL
find_package(PCL 1.7 REQUIRED COMPONENTS
    common
    io
    filters
    features
    segmentation
)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
### Eigen
find_package(Eigen3 REQUIRED)

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
    CATKIN_DEPENDS object_builders_lib
    #  DEPENDS system_lib
    INCLUDE_DIRS include
    LIBRARIES segmenters_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

## Declare a C++ executable
add_library(${PROJECT_NAME}
    src/ground_plane_fitting_segmenter.cpp
    src/ground_ransac_segmenter.cpp
    src/euclidean_segmenter.cpp
    src/region_euclidean_segmenter.cpp
    src/region_growing_segmenter.cpp
    src/don_segmenter.cpp
)

## http://mariobadr.com/creating-a-header-only-library-with-cmake.html
#add_library(${PROJECT_NAME} INTERFACE)
#target_include_directories(${PROJECT_NAME} INTERFACE include/common)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Eigen3_LIBRARIES}
)

#############
## Install ##
#############

# ...


#############
## Samples ##
#############

## Comment following line to avoid generating demo node
add_subdirectory(samples)