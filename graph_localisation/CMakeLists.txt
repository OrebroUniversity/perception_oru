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
    ndt_registration
    ndt_map
    ndt_mcl
    ndt_localization
    graph_map
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
find_package(Boost REQUIRED COMPONENTS signals)
catkin_package(
    DEPENDS Eigen libpcl-all-dev
    CATKIN_DEPENDS ndt_mcl graph_map ndt_registration ndt_map roscpp laser_geometry sensor_msgs tf_conversions tf geometry_msgs cmake_modules eigen_conversions pcl_ros
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    )


include_directories(include  ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set (${PROJECT_NAME}_LIB_SRCS
    src/localisation_type.cpp
    src/localisation_factory.cpp
    src/mcl_ndt/mcl_ndt.cpp
    )
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
add_executable(test_localisation test/test_localisation.cpp )
target_link_libraries(test_localisation ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

add_executable(graph_localisation_node src/graph_localisation_node.cpp)
target_link_libraries(graph_localisation_node ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})



##
#
#target_link_libraries(show_map ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})


#  ${Eigen_LIBRARIES}
#)

#
    install(DIRECTORY include
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      )
