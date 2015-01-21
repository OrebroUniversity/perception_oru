cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE Release) 
project(sdf_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs cv_bridge)

catkin_package(
    DEPENDS eigen opencv2 libvtk
    CATKIN_DEPENDS roscpp std_msgs cv_bridge
    INCLUDE_DIRS include
    LIBRARIES # TODO
)
find_package(VTK REQUIRED)
add_executable(sdf_tracker_node src/sdf_tracker_node.cpp)
add_library(${PROJECT_NAME} src/sdf_tracker.cpp)

find_package(OpenMP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

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

include_directories(${EIGEN_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS} include)
add_definitions(${EIGEN_DEFINITIONS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${VTK_LIBRARIES})
target_link_libraries(sdf_tracker_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME})

#target_link_libraries(example ${PROJECT_NAME})
## Generate added messages and services with any dependencies listed here
#generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
install(TARGETS sdf_tracker_node ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )
install(FILES
   launch/launch_sdf_tracker.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

