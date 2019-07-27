cmake_minimum_required(VERSION 2.8.3)
project(gpd_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cmake_modules eigen_conversions message_generation roscpp rospy sensor_msgs std_msgs)

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Libtorch
set(CMAKE_PREFIX_PATH "/home/sdhm/libtorch/share/cmake")
find_package(Torch)
message(STATUS "Pytorch status:")
message(STATUS "libraries: ${TORCH_LIBRARIES}")

## System dependencies are found with CMake's conventions
find_library(GPD_LIBRARY NAME gpd PATHS /usr/local/lib PATH_SUFFIXES lib NO_DEFAULT_PATH)
if (GPD_LIBRARY)
  message(STATUS "Library GPD found in ${GPD_LIBRARY}")
else()
  message(FATAL_ERROR "Library GPD not found")
endif()
include_directories(${GPD_LIBRARY_INCLUDE_DIR})
message(STATUS "gpd_include_dir: ${GPD_LIBRARY_INCLUDE_DIR}")

find_library(GPD_POINTNET_LIBRARY NAMES gpd_pointnet PATHS /usr/local/lib PATH_SUFFIXES lib NO_DEFAULT_PATH)
if (GPD_POINTNET_LIBRARY)
  message(STATUS "Library GPD_POINTNET found in ${GPD_POINTNET_LIBRARY}")
else()
  message(FATAL_ERROR "Library GPD_POINTNET not found")
endif()
include_directories(${GPD_POINTNET_LIBRARY_INCLUDE_DIR})
message(STATUS "gpd_pointnet_include_dir: ${GPD_POINTNET_LIBRARY_INCLUDE_DIR}")


set(CMAKE_CXX_FLAGS "-O3 -fopenmp -fPIC -Wno-deprecated -Wenum-compare -Wno-ignored-attributes -std=c++14")

## Generate messages in the 'msg' folder
add_message_files(FILES CloudIndexed.msg CloudSamples.msg CloudSources.msg GraspConfig.msg GraspConfigList.msg
  SamplesMsg.msg)

## Generate services in the 'srv' folder
add_service_files(FILES detect_grasps.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

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
CATKIN_DEPENDS cmake_modules eigen_conversions geometry_msgs message_runtime roscpp sensor_msgs std_msgs
DEPENDS PCL
# CATKIN_DEPENDS cmake_modules eigen_conversions geometry_msgs message_runtime roscpp sensor_msgs std_msgs
# DEPENDS Eigen OpenCV PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

## Declare a C++ library
add_library(${PROJECT_NAME}_grasp_messages src/${PROJECT_NAME}/grasp_messages.cpp)
add_library(${PROJECT_NAME}_grasp_plotter src/${PROJECT_NAME}/grasp_plotter.cpp)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_detect_grasps src/gpd_ros/grasp_detection_node.cpp)
add_executable(${PROJECT_NAME}_detect_grasps_pointnet src/gpd_ros/grasp_detection_pointnet.cpp)
add_executable(${PROJECT_NAME}_detect_grasps_server src/gpd_ros/grasp_detection_server.cpp)

## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_detect_grasps PROPERTIES OUTPUT_NAME detect_grasps PREFIX "")
set_target_properties(${PROJECT_NAME}_detect_grasps_pointnet PROPERTIES OUTPUT_NAME detect_grasps_pointnet PREFIX "")
set_target_properties(${PROJECT_NAME}_detect_grasps_server PROPERTIES OUTPUT_NAME detect_grasps_server PREFIX "")

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_detect_grasps
                      ${PROJECT_NAME}_grasp_messages
                      ${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIBRARY}
                      ${PCL_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${TORCH_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_detect_grasps_pointnet
                      ${PROJECT_NAME}_grasp_messages
                      ${PROJECT_NAME}_grasp_plotter
                      ${GPD_POINTNET_LIBRARY}
                      ${PCL_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${TORCH_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_detect_grasps_server
                      ${PROJECT_NAME}_grasp_messages
                      ${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIBRARY}
                      ${PCL_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${TORCH_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIBRARY}
                      ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_grasp_messages
                      ${GPD_LIBRARY}
                      ${catkin_LIBRARIES})