# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(ndt_registration)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS ndt_map pcl pointcloud_vrml tf tf_conversions eigen)
catkin_package(
    DEPENDS ndt_map pcl pointcloud_vrml tf tf_conversions eigen
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include
    LIBRARIES # TODO
)

find_package(Boost REQUIRED COMPONENTS program_options)
#include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} )

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

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

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} ${LIBRARY_SOURCES})
#target_link_libraries(${PROJECT_NAME} another_library)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#add_executable(ndt_registration_test test/ndt_registration_test.cc)
#rosbuild_add_openmp_flags(ndt_registration_test)

find_package(OpenMP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

add_executable(simple_test test/simple.cc)
target_link_libraries(simple_test ${Boost_LIBRARIES} ${catkin_LIBRARIES})

#rosbuild_add_openmp_flags(simple_test)


add_executable(from_jff test/from_jff.cc)
target_link_libraries(from_jff ${Boost_LIBRARIES} ${catkin_LIBRARIES})
#rosbuild_add_openmp_flags(from_jff)

#target_link_libraries(ndt_registration_test ndt_map pointcloud_vrml)

add_executable(ndt_feature_test test/ndt_feature_test.cc)
target_link_libraries(ndt_feature_test ${Boost_LIBRARIES} ${catkin_LIBRARIES})

#target_link_libraries(ndt_feature_test ${Boost_LIBRARIES})

#target_link_libraries(ndt_feature_test ${PROJECT_NAME} ndt_map pointcloud_vrml)

#add_executable(feature_registration test/feature_registration.cc)
#target_link_libraries(feature_registration ndt_map pointcloud_vrml )

#add_executable(ndt_reconstruct test/reconstruct.cc)
#target_link_libraries(ndt_reconstruct ${PROJECT_NAME} ndt_map pointcloud_vrml )

#add_executable(ndt_matcher_node test/ndt_matcher_node.cc)
#target_link_libraries(ndt_matcher_node ${PROJECT_NAME} ndt_map pointcloud_vrml )

#add_executable(batchTestRangerAccuracy test/batchTestRangerAccuracy.cc)
#target_link_libraries(batchTestRangerAccuracy ${PROJECT_NAME} ndt_map  pointcloud_vrml)

#add_executable(narf_keypoint_extraction test/narf_extraction.cc)
#target_link_libraries(narf_keypoint_extraction ${PROJECT_NAME} ndt_map pointcloud_vrml )

#add_executable(narf_registration test/narf_registration.cc)
#target_link_libraries(narf_registration ${PROJECT_NAME} ndt_map pointcloud_vrml )

#add_executable(ndt_histogram_registration test/ndt_histogram_registration.cc)
#target_link_libraries(ndt_histogram_registration ${PROJECT_NAME} ndt_map pointcloud_vrml )

#add_executable(test_covariance test/test_covariance.cc)
#target_link_libraries(test_covariance ${PROJECT_NAME} ndt_map pointcloud_vrml )

#target_link_libraries(test_ndt_mapper ${PROJECT_NAME} ndt_map pointcloud_vrml )

## Generate added messages and services with any dependencies listed here
#generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
