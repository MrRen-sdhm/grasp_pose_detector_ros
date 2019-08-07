cmake_minimum_required(VERSION 2.8.3)
project(gpd_ros)

set(CMAKE_CXX_FLAGS "-O3 -fopenmp -fPIC -Wno-deprecated -Wenum-compare -Wno-ignored-attributes -std=c++14")

# Find catkin macros and libraries
#find_package(catkin REQUIRED COMPONENTS cmake_modules eigen_conversions message_generation roscpp rospy sensor_msgs
#        std_msgs message_filters cv_bridge image_transport compressed_image_transport compressed_depth_image_transport)
# Opencv 3.4 配置, cv相关的库不使用catkin查找
find_package(catkin REQUIRED COMPONENTS cmake_modules eigen_conversions message_generation roscpp rospy sensor_msgs
        std_msgs message_filters)
find_package(cv_bridge)
find_package(image_transport)
find_package(compressed_image_transport)
find_package(compressed_depth_image_transport)

# Opencv3.4
find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBRARIES}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# OpenMP
find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

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
message(STATUS "catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")

## Declare a C++ library
add_library(${PROJECT_NAME}_grasp_messages src/${PROJECT_NAME}/grasp_messages.cpp)
add_library(${PROJECT_NAME}_grasp_plotter src/${PROJECT_NAME}/grasp_plotter.cpp)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_detect_grasps src/gpd_ros/grasp_detection_node.cpp)
add_executable(${PROJECT_NAME}_detect_grasps_pointnet src/gpd_ros/grasp_detection_pointnet.cpp)
add_executable(${PROJECT_NAME}_detect_grasps_pointnet_realsense src/gpd_ros/grasp_detection_pointnet_realsense.cpp)
add_executable(${PROJECT_NAME}_detect_grasps_yolo src/gpd_ros/grasp_detection_yolo.cpp)
add_executable(${PROJECT_NAME}_detect_grasps_server src/gpd_ros/grasp_detection_server.cpp)

## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_detect_grasps PROPERTIES OUTPUT_NAME detect_grasps PREFIX "")
set_target_properties(${PROJECT_NAME}_detect_grasps_pointnet PROPERTIES OUTPUT_NAME detect_grasps_pointnet PREFIX "")
set_target_properties(${PROJECT_NAME}_detect_grasps_pointnet_realsense PROPERTIES OUTPUT_NAME detect_grasps_pointnet_realsense PREFIX "")
set_target_properties(${PROJECT_NAME}_detect_grasps_yolo PROPERTIES OUTPUT_NAME detect_grasps_yolo PREFIX "")
set_target_properties(${PROJECT_NAME}_detect_grasps_server PROPERTIES OUTPUT_NAME detect_grasps_server PREFIX "")

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_detect_grasps
                      ${PROJECT_NAME}_grasp_messages
                      ${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIBRARY}
                      ${PCL_LIBRARIES}
                      ${TORCH_LIBRARIES}
#                      ${catkin_LIBRARIES}
                      ${image_transport_LIBRARIES}
                      ${cv_bridge_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_detect_grasps_pointnet
                      ${PROJECT_NAME}_grasp_messages
                      ${PROJECT_NAME}_grasp_plotter
                      ${GPD_POINTNET_LIBRARY}
                      ${PCL_LIBRARIES}
                      ${TORCH_LIBRARIES}
#                      ${catkin_LIBRARIES}
                      ${image_transport_LIBRARIES}
                      ${cv_bridge_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_detect_grasps_pointnet_realsense
                      ${PROJECT_NAME}_grasp_messages
                      ${PROJECT_NAME}_grasp_plotter
                      ${GPD_POINTNET_LIBRARY}
                      ${PCL_LIBRARIES}
                      ${TORCH_LIBRARIES}
#                      ${catkin_LIBRARIES}
                      ${image_transport_LIBRARIES}
                      ${cv_bridge_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_detect_grasps_yolo
                      ${PROJECT_NAME}_grasp_messages
                      ${PROJECT_NAME}_grasp_plotter
                      ${GPD_POINTNET_LIBRARY}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${TORCH_LIBRARIES}
#                      ${catkin_LIBRARIES}
                      ${image_transport_LIBRARIES}
                      ${cv_bridge_LIBRARIES}
                      )
target_link_libraries(${PROJECT_NAME}_detect_grasps_server
                      ${PROJECT_NAME}_grasp_messages
                      ${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIBRARY}
                      ${PCL_LIBRARIES}
                      ${TORCH_LIBRARIES}
#                      ${catkin_LIBRARIES}
                      ${image_transport_LIBRARIES}
                      ${cv_bridge_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIBRARY}
                      ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_grasp_messages
                      ${GPD_LIBRARY}
                      ${catkin_LIBRARIES})
