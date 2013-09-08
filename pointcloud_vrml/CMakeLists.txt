# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_vrml)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS 
#common_rosdeps <- it seams that it is not necessary anymore 
rosconsole
pcl pcl_ros tf nav_msgs eigen)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#find_package(Eigen REQUIRED)
#find_package(pcl REQUIRED)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#add_message_files(
#  FILES
  # TODO: List your msg files here
#)
#uncomment if you have defined services
#add_service_files(
#  FILES
  # TODO: List your msg files here
#)

#find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS}  ${catkin_INCLUDE_DIRS}  include)
add_definitions(${EIGEN_DEFINITIONS})


add_executable(vrml_reader src/pointcloud_reader_node.cc)
add_executable(vrml_writer src/pointcloud_writer_node.cc)
add_executable(logConverter test/processLogDir.cc)
add_executable(subsample test/subsampleRandom.cc)
add_executable(rpy2q test/rpy2q.cc)
add_executable(trajectory test/compareTrajectory.cc)
target_link_libraries(trajectory ${catkin_LIBRARIES})
target_link_libraries(vrml_reader ${catkin_LIBRARIES})
target_link_libraries(vrml_writer ${catkin_LIBRARIES})

## Generate added messages and services with any dependencies listed here
generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS 
#common_rosdeps 
pcl pcl_ros tf nav_msgs eigen
rosconsole
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
