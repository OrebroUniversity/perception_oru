# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE Release) 
project(ndt_map)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS pcl_ros pointcloud_vrml cv_bridge eigen 
  std_msgs
   message_generation  )

add_message_files(
  FILES
  NDTMap.msg   
  NDTCell.msg
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(oct_builder test/octTreeBuilder.cc)
add_executable(histTest test/test_ndt_histogram.cc)
add_executable(jffSaveTest test/jffLazyGridTest_saving.cc)
add_executable(jffLoadTest test/jffLazyGridTest_loading.cc)
add_executable(test_map_topic test/test_map_topic.cpp)
add_executable(test_map_topic_1 test/test_map_topic_1.cpp)

find_package(OpenCV REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS})

#uncomment if you have defined messages

#uncomment if you have defined services
#add_service_files(
#  FILES
  # TODO: List your msg files here
#)

#make a node that listens to point clouds and makes ndts

target_link_libraries(oct_builder ${catkin_LIBRARIES})
target_link_libraries(histTest ${catkin_LIBRARIES})
target_link_libraries(jffSaveTest ${catkin_LIBRARIES})
target_link_libraries(jffLoadTest ${catkin_LIBRARIES})
target_link_libraries(test_map_topic ${catkin_LIBRARIES})
target_link_libraries(test_map_topic_1 ${catkin_LIBRARIES})
#add_executable(likelihood_test test/ndtLikelihoodTester.cc)
#target_link_libraries(likelihood_test ${PROJECT_NAME}  pointcloud_vrml)
##add_executable(ltest test/likelihoodSingleScan.cc)
#add_executable(batchTestHist test/batchTesterHistogramFPFH.cc)
#target_link_libraries(batchTestHist ${PROJECT_NAME} pointcloud_vrml)
## Generate added messages and services with any dependencies listed here

catkin_package(
    DEPENDS eigen opencv2 libpcl-all-dev
    CATKIN_DEPENDS pcl_ros pointcloud_vrml cv_bridge 
    INCLUDE_DIRS include
    LIBRARIES # ${PROJECT_NAME}
)

install(TARGETS oct_builder 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )

