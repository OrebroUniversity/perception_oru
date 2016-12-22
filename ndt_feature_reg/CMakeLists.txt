cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE Release) 
project(ndt_feature_reg)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS cmake_modules rosconsole ndt_registration ndt_map laser_geometry sensor_msgs std_srvs image_geometry cv_bridge image_transport)
catkin_package(
    DEPENDS eigen libpcl-all-dev #mrpt glut libxmu-dev  
    CATKIN_DEPENDS ndt_registration ndt_map laser_geometry sensor_msgs ndt_visualisation std_srvs image_geometry cv_bridge image_transport
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

###############################################################################################################################
#NOTE: There are two options for compiling this
#1) 
# In order to fix : 'opencv2/nonfree/features2d.hpp no such file or directory' type of errors add the following pkg.
#
# sudo add-apt-repository --yes ppa:xqms/opencv-nonfree
# sudo apt-get update 
# sudo apt-get install libopencv-nonfree-dev
#
#2) 
#install a local copy of openCV with the non-free modules built
#for ubuntu 14.04, do the following:
#mkdir tmp && cd tmp && wget  https://raw.githubusercontent.com/jayrambhia/Install-OpenCV/master/Ubuntu/2.4/opencv2_4_9.sh
#chmod a+x opencv2_4_9.sh
#edit opencv2_4_9.sh. Add -D WITH_OPENCL=OFF -D BUILD_opencv_nonfree=ON to the cmake command on line 30
#run the script and verify you have installed the opencv library in /usr/local

###############################################################################################################################
set(OpenCV_DIR "/usr/local/")
find_package(OpenCV REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})
ADD_DEFINITIONS(-DLINUX_OS)
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(VTK REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

set (${PROJECT_NAME}_LIB_SRCS
	src/ndt_frame_proc.cpp
	src/ndt_frame.cpp
	src/ndt_frame_viewer.cpp
	)

link_directories(/usr/local/lib)

#uncomment these lines to build project binaries
#add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_executable(ndt_feature_reg_node src/ndt_feature_reg_node.cc)
#target_link_libraries(ndt_feature_reg_node ${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_executable(ndt_feature_eval src/ndt_feature_eval.cc)
#target_link_libraries(ndt_feature_eval ${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_executable(ndt_feature_viewer src/ndt_feature_viewer.cc)
#target_link_libraries(ndt_feature_viewer ${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBRARIES})

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
install(TARGETS #ndt_feature_reg_node ${PROJECT_NAME} ndt_feature_eval
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
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/eval/
)
install(DIRECTORY src/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src/
)
