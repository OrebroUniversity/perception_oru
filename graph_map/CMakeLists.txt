cmake_minimum_required(VERSION 2.8.3)
project(graph_map)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE release)

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
add_definitions(-std=c++11)
endif()

find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED
    geometry_msgs
    cmake_modules
    ndt_registration
    ndt_map
    ndt_mcl
    ndt_localization
    laser_geometry
    eigen_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    ndt_map
    tf_conversions
    tf
    genmsg
    )
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED COMPONENTS)


catkin_package(
    DEPENDS Eigen libpcl-all-dev
    CATKIN_DEPENDS ndt_mcl ndt_registration ndt_map roscpp laser_geometry sensor_msgs tf_conversions tf geometry_msgs cmake_modules eigen_conversions pcl_ros
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    )


include_directories(include  ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set (${PROJECT_NAME}_LIB_SRCS
    src/graph_map/graph_map.cpp
    src/graph_map/graph_map_navigator.cpp
    src/graph_map/map_node.cpp
    src/graph_map/map_type.cpp
    src/graph_map_fuser.cpp
    src/graph_map/reg_type.cpp
    src/ndt/ndt_map_type.cpp
    src/ndt/ndtd2d_reg_type.cpp
    src/ndt/ndt_map_param.cpp
    src/graphfactory.cpp
    src/visualization/graph_plot.cpp
    src/lidarUtils/lidar_utilities.cpp
    src/template/template_map_type.cpp
    src/template/template_reg_type.cpp
    src/graph_map/factor.cpp
    src/ndt_dl/ndtdl_map_type.cpp
    src/ndt_dl/ndtdl_reg_type.cpp
    )
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
add_executable(test_graph test/test_graph.cpp )
add_executable(test_particle_filter test/test_particle_filter.cpp )
add_executable(test_serialization test/test_serialization.cpp )
add_executable(show_map src/show_map.cpp )
add_executable(graph_map_fuser_node src/graph_map_fuser_node.cpp )



##
#
target_link_libraries(show_map ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
target_link_libraries(test_graph ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
target_link_libraries(test_particle_filter ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
target_link_libraries(test_serialization ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
target_link_libraries(graph_map_fuser_node ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

#  ${Eigen_LIBRARIES}
#)

#
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
