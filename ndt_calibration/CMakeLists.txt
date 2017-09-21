cmake_minimum_required(VERSION 2.8.3)
project(ndt_calibration)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ndt_fuser
  ndt_map
  ndt_registration
  ndt_visualisation
  ndt_rviz
  ndt_mcl
  ndt_offline
  ndt_generic
  pcl_ros
  pcl_conversions
  tf
  tf_conversions
  eigen_conversions
  velodyne_msgs
  velodyne_pointcloud
  roscpp
  angles
)

find_package( PkgConfig REQUIRED)
pkg_check_modules( gsl REQUIRED gsl )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ndt_calibration
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS gsl
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

set (${PROJECT_NAME}_LIB_SRCS
	    src/ndt_calib.cpp
	)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${gsl_LIBRARIES})

add_executable(ndt_calib src/ndt_calib_main.cpp)
target_link_libraries(ndt_calib ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${gsl_LIBRARIES} ${PROJECT_NAME})

add_executable(ndt_calib_test src/ndt_calib_test.cpp)
target_link_libraries(ndt_calib_test ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${gsl_LIBRARIES} ${PROJECT_NAME})

add_executable(ndt_calib_online_node src/ndt_calib_online_node.cpp)
target_link_libraries(ndt_calib_online_node ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${gsl_LIBRARIES} ${PROJECT_NAME})


#############
## Install ##
#############

#############
## Testing ##
#############
