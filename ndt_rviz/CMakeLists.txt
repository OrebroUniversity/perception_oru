cmake_minimum_required(VERSION 2.8.3)
project(ndt_rviz)

find_package(catkin REQUIRED COMPONENTS cmake_modules)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ndt_rviz
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

install(DIRECTORY include/${PROJECT_NAME}/
	    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

