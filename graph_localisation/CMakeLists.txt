cmake_minimum_required(VERSION 2.8.3)
project(graph_localisation)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE release)

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
add_definitions(-std=c++11)
endif()

find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED
    geometry_msgs
    cmake_modules
    laser_geometry
    eigen_conversions
    pcl_ros
    ndt_generic
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
    CATKIN_DEPENDS roscpp cmake_modules ndt_generic laser_geometry sensor_msgs tf_conversions tf geometry_msgs cmake_modules eigen_conversions pcl_ros
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    )


include_directories(include  ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set (${PROJECT_NAME}_LIB_SRCS
    src/localisation_type.cpp
    src/localisation_factory.cpp
    )
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
add_executable(test_localisation test/test_localisation.cpp )
target_link_libraries(test_localisation ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})


##
#
#target_link_libraries(show_map ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})


#  ${Eigen_LIBRARIES}
#)

#
    install(DIRECTORY include/${PROJECT_NAME}/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      FILES_MATCHING PATTERN "*.h"
      PATTERN ".svn" EXCLUDE)
