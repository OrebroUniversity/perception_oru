# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE Release) 
project(ndt_map_builder)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS pcl_ros pointcloud_vrml ndt_map ndt_registration)

catkin_package(
    DEPENDS pcl eigen
    CATKIN_DEPENDS pcl_ros pointcloud_vrml ndt_map ndt_registration
    INCLUDE_DIRS include
    LIBRARIES # TODO
)
include_directories(include ${catkin_INCLUDE_DIRS})

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

find_package(OpenMP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

add_executable(test_ndt_mapper test/test_ndt_mapper.cc)
target_link_libraries(test_ndt_mapper ${catkin_LIBRARIES})

## Generate added messages and services with any dependencies listed here
#generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

install(TARGETS test_ndt_mapper 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )
