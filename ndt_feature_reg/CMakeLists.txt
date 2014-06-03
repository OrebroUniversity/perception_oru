cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE Release) 
project(ndt_feature_reg)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rosconsole ndt_registration ndt_map laser_geometry sensor_msgs std_srvs image_geometry cv_bridge OpenCV)
catkin_package(
    DEPENDS eigen libpcl-all-dev #mrpt glut libxmu-dev  
    CATKIN_DEPENDS ndt_registration ndt_map laser_geometry sensor_msgs ndt_visualisation std_srvs image_geometry cv_bridge
    INCLUDE_DIRS include
    LIBRARIES
)


include_directories(${catkin_INCLUDE_DIRS})
ADD_DEFINITIONS(-DLINUX_OS)
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(VTK REQUIRED)


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

include_directories(${Boost_INCLUDE_DIRS})

add_executable(ndt_feature_reg src/ndt_feature_reg_node.cc)
target_link_libraries(ndt_feature_reg ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(ndt_feature_eval src/ndt_feature_eval.cc)
target_link_libraries(ndt_feature_eval ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(ndt_feature_viewer src/ndt_feature_viewer.cc)
target_link_libraries(ndt_feature_viewer ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${VTK_LIBRARIES})

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
install(TARGETS ndt_feature_reg ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(TARGETS ndt_feature_eval ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )
install(FILES
   launch/replay_log_TUM_freiburg1_desk_ndt_feature_reg_node.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY eval/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
