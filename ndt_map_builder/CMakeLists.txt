# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE Release) 
project(ndt_map_builder)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS pcl_ros pcl_conversions ndt_map ndt_registration)

catkin_package(
    DEPENDS libpcl-all-dev eigen
    CATKIN_DEPENDS pcl_ros ndt_map ndt_registration pcl_conversions 
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} # TODO
)
set (${PROJECT_NAME}_LIB_SRCS
  src/ndt_map_builder.cpp
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
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
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -std=c++0x -ggdb")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

add_executable(test_ndt_mapper test/test_ndt_mapper.cc)
include_directories(include ${PCL_INCLUDE_DIRS})

target_link_libraries(test_ndt_mapper ${PROJECT_NAME} ${catkin_LIBRARIES})
link_directories(${catkin_LIBRARY_DIRS})
## Generate added messages and services with any dependencies listed here
#generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

install(TARGETS  ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )
