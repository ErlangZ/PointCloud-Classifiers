cmake_minimum_required(VERSION 2.8.7)
project(PointCloud-Util C CXX)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)
pkg_search_module(Eigen3 REQUIRED eigen3)
set(USE_EIGEN 1)
include_directories(${EIGEN3_INCLUDE_DIR})
link_directories(${EIGEN3_LIBRARY_DIRS})

set(PCL_LIBS pcl_io pcl_common pcl_visualization pcl_filters pcl_sample_consensus) 

file(GLOB _SRCS "*.cpp")
add_library(${PROJECT_NAME} ${_SRCS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBS} pthread boost_system)

add_executable(${PROJECT_NAME}_main ${_SRCS})
target_link_libraries(${PROJECT_NAME}_main ${OpenCV_LIBS} ${PCL_LIBS} pthread boost_system)



