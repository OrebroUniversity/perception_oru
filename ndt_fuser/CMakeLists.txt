# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE Release) 
project(ndt_fuser)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rosconsole ndt_registration ndt_map laser_geometry sensor_msgs ndt_visualisation std_srvs)
catkin_package(
    DEPENDS eigen pcl mrpt glut libxmu-dev 
    CATKIN_DEPENDS ndt_registration ndt_map laser_geometry sensor_msgs ndt_visualisation std_srvs
    INCLUDE_DIRS include
    LIBRARIES # TODO
)
add_executable(ndt_fuser src/ndt_fuser_node.cpp)

find_package(LibXml2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
ADD_DEFINITIONS(-DLINUX_OS)
#find_package(ndt_visualisation)
find_package(OpenGL REQUIRED)
find_package(GLU REQUIRED)
find_package(GLUT REQUIRED)
find_package(JPEG REQUIRED)
include_directories(${JPEG_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(include)
FIND_PACKAGE(MRPT REQUIRED gui;base;opengl)
#find_package(MRPT REQUIRED)
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

include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#message(STATUS "->" ${MRPT_LIBS})
#message(STATUS "->" ${MRPT_LIBRARIES})
target_link_libraries(ndt_fuser ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${MRPT_LIBS} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(test_ndt_fuser src/ndt_fuser_from_depth.cpp)
#target_link_libraries(test_ndt_fuser ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${MRPT_LIBS})
#target_link_libraries(test_ndt_fuser ${Boost_LIBRARIES})
#rosbuild_add_openmp_flags(test_ndt_fuser)

#target_link_libraries(example ${PROJECT_NAME})
## Generate added messages and services with any dependencies listed here
#generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
install(TARGETS ndt_fuser ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )
