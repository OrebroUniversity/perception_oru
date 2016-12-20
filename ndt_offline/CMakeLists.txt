cmake_minimum_required(VERSION 2.8.3)
project(ndt_offline)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ndt_fuser
  ndt_map
  ndt_registration
  ndt_visualisation
  pcl_ros
  velodyne_msgs
  velodyne_pointcloud
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ndt_fuser ndt_map ndt_registration ndt_visualisation pcl_ros velodyne_msgs velodyne_pointcloud
  DEPENDS eigen libpcl-all-dev mrpt glut libxmu-dev 
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(LibXml2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
ADD_DEFINITIONS(-DLINUX_OS)
#find_package(ndt_visualisation)
find_package(OpenGL REQUIRED)
find_package(GLU REQUIRED)
find_package(GLUT REQUIRED)
find_package(JPEG REQUIRED)
include_directories(${JPEG_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(include)
find_package(OpenMP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

add_executable(fuser3d_offline src/fuser3d_offline.cpp)
target_link_libraries(fuser3d_offline ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
