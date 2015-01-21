# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE Release) 
project(ndt_visualisation)
# Load catkin and all dependencies required for this package
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS ndt_map pcl_ros)
catkin_package(
    DEPENDS libpcl-all-dev eigen glut opengl#libxmu-dev mrpt 
    CATKIN_DEPENDS ndt_map pcl_ros message_runtime
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)
set(CMAKE_CXX_FLAGS "-fPIC ${CMAKE_CXX_FLAGS}")

#find_package(LibXml2 REQUIRED)
#include_directories(${LIBXML2_INCLUDE_DIR})
#ADD_DEFINITIONS(-DLINUX_OS)
#find_package(GLU REQUIRED)
#find_package(JPEG REQUIRED)
#FIND_PACKAGE(MRPT REQUIRED gui;base;opengl)
#include_directories(${JPEG_INCLUDE_DIR})

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
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
find_package(Boost REQUIRED COMPONENTS program_options thread)
include_directories(${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME}  src/NDTVizGlut.cc)
target_link_libraries(${PROJECT_NAME} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${catkin_LIBRARIES}) #${JPEG_LIBRARIES} ${LIBXML2_LIBRARIES} ${MRPT_LIBS} 


add_executable(ndt_viewer src/ndt_viewer.cc)
target_link_libraries(ndt_viewer ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME}) # ${MRPT_LIBS} ${JPEG_LIBRARIES} ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} 

#add_executable(glut_test src/glut_test.cc)
#target_link_libraries(glut_test ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# add_executable(glut3d_test src/glut3d_test.cc)
# target_link_libraries(glut3d_test ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME})

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
#install(TARGETS ndt_viewer ${PROJECT_NAME}
install(TARGETS ndt_viewer ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )
