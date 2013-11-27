# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(ndt_map)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS pcl pcl_ros pointcloud_vrml cv_bridge eigen)
catkin_package(
    DEPENDS pcl pcl_ros pointcloud_vrml opencv2 cv_bridge eigen
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include
    LIBRARIES # ${PROJECT_NAME}
)

add_executable(oct_builder test/octTreeBuilder.cc)
add_executable(histTest test/test_ndt_histogram.cc)
add_executable(jffSaveTest test/jffLazyGridTest_saving.cc)
add_executable(jffLoadTest test/jffLazyGridTest_loading.cc)

find_package(OpenCV REQUIRED)
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

#make a node that listens to point clouds and makes ndts

target_link_libraries(oct_builder ${catkin_LIBRARIES})
target_link_libraries(histTest ${catkin_LIBRARIES})
target_link_libraries(jffSaveTest ${catkin_LIBRARIES})
target_link_libraries(jffLoadTest ${catkin_LIBRARIES})
#add_executable(likelihood_test test/ndtLikelihoodTester.cc)
#target_link_libraries(likelihood_test ${PROJECT_NAME}  pointcloud_vrml)
##add_executable(ltest test/likelihoodSingleScan.cc)
#add_executable(batchTestHist test/batchTesterHistogramFPFH.cc)
#target_link_libraries(batchTestHist ${PROJECT_NAME} pointcloud_vrml)
## Generate added messages and services with any dependencies listed here
#generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
