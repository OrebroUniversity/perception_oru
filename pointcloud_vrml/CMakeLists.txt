# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE Release) 
project(pointcloud_vrml)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rosconsole pcl_ros tf nav_msgs eigen)
find_package(PCL REQUIRED)
catkin_package(
    DEPENDS pcl eigen
    CATKIN_DEPENDS rosconsole pcl_ros tf nav_msgs
    INCLUDE_DIRS include
    LIBRARIES 
)

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

include_directories(${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}  include)
add_definitions(${EIGEN_DEFINITIONS})

add_executable(vrml_reader src/pointcloud_reader_node.cc)
add_executable(vrml_writer src/pointcloud_writer_node.cc)
#add_executable(logConverter test/processLogDir.cc)
#add_executable(subsample test/subsampleRandom.cc)
#add_executable(rpy2q test/rpy2q.cc)
#add_executable(trajectory test/compareTrajectory.cc)
#target_link_libraries(trajectory ${catkin_LIBRARIES})
target_link_libraries(vrml_reader ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(vrml_writer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS vrml_reader vrml_writer
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )


## Generate added messages and services with any dependencies listed here
#generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

