cmake_minimum_required(VERSION 2.8.3)
project(ndt_generic)

find_package(catkin REQUIRED COMPONENTS cmake_modules angles)
find_package(Boost REQUIRED COMPONENTS serialization filesystem)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ndt_generic
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

include_directories(include ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(eigen_test test/eigen_test.cpp)
target_link_libraries(eigen_test
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )
