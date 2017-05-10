cmake_minimum_required(VERSION 2.8.3)
project(ndt_generic)

find_package(catkin REQUIRED COMPONENTS cmake_modules angles)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(Eigen3 REQUIRED)

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
