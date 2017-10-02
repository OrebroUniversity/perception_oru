cmake_minimum_required(VERSION 2.8.3)
project(ndt_offline)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_BUILD_TYPE release    )
add_definitions(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  ndt_fuser
  ndt_map
  ndt_registration
  ndt_visualisation
  ndt_rviz
  ndt_mcl
  ndt_generic
  pcl_ros
  velodyne_msgs
  velodyne_pointcloud
  tf_conversions
  eigen_conversions
  graph_map
  graph_localisation
  tf
  genmsg
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ndt_fuser ndt_mcl ndt_map ndt_registration ndt_visualisation pcl_ros velodyne_msgs velodyne_pointcloud graph_map graph_localisation tf tf_conversions genmsg
  DEPENDS eigen libpcl-all-dev mrpt glut libxmu-dev 
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(LibXml2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals)
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
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

FIND_PACKAGE(VTK REQUIRED NO_MODULE)
IF (VTK_FOUND)
  INCLUDE (${VTK_USE_FILE})
ELSE(VTK_FOUND)
  MESSAGE(FATAL_ERROR "Cannot build the executable without VTK. Please set the VTK_DIR")
ENDIF (VTK_FOUND)

include_directories(
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

add_executable(fuser3d_offline src/fuser3d_offline.cpp)
target_link_libraries(fuser3d_offline ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})


add_executable(bag_converter src/bag_converter.cpp)
target_link_libraries(bag_converter ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})

add_executable(bag_file_view src/bag_file_view.cpp)
target_link_libraries(bag_file_view ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})

add_executable(mcl_offline src/mcl_offline.cpp)
target_link_libraries(mcl_offline ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})

add_executable(graph_mcl_offline src/graph_mcl_offline.cpp)
target_link_libraries(graph_mcl_offline ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})

add_executable(graph_fuser3d_offline src/graph_fuser3d_offline.cpp)
target_link_libraries(graph_fuser3d_offline ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})


add_executable(view_ndtmap src/view_ndtmap.cpp)
target_link_libraries(view_ndtmap ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(ndt_offline_laser src/ndt_offline_laser.cpp)
target_link_libraries(ndt_offline_laser ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} )

