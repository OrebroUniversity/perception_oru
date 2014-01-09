# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE Release) 
project(ndt_visualisation)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS ndt_map pcl_ros)
catkin_package(
    DEPENDS libpcl-all-dev eigen mrpt glut libxmu-dev
    CATKIN_DEPENDS ndt_map pcl_ros
    INCLUDE_DIRS include
    LIBRARIES ndt_visualisation
)

find_package(LibXml2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})
ADD_DEFINITIONS(-DLINUX_OS)
find_package(OpenGL REQUIRED)
find_package(GLU REQUIRED)
find_package(GLUT REQUIRED)
find_package(JPEG REQUIRED)
FIND_PACKAGE(MRPT REQUIRED gui;base;opengl)
include_directories(${JPEG_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

#FIND_PACKAGE(MRPT REQUIRED graphslam;gui)

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
add_library(${PROJECT_NAME} src/CMyEllipsoid.cpp)
target_link_libraries(${PROJECT_NAME} ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${MRPT_LIBS} ${catkin_LIBRARIES})

find_package(Boost REQUIRED COMPONENTS program_options thread)
include_directories(${Boost_INCLUDE_DIRS})
add_executable(ndt_viewer src/ndt_viewer.cc)
target_link_libraries(ndt_viewer ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${MRPT_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME})


#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#target_link_libraries(example ${PROJECT_NAME})
## Generate added messages and services with any dependencies listed here
#generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
install(TARGETS ndt_viewer ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )
