# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE Release) 
project(ndt_mcl)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS ndt_visualisation rosbag tf nav_msgs sensor_msgs geometry_msgs message_filters ndt_map ndt_registration pointcloud_vrml visualization_msgs)

catkin_package(
    DEPENDS libpcl-all-dev eigen mrpt
    CATKIN_DEPENDS ndt_visualisation rosbag tf nav_msgs sensor_msgs geometry_msgs message_filters ndt_map ndt_registration pointcloud_vrml visualization_msgs
    INCLUDE_DIRS #include
    LIBRARIES # TODO
)
include_directories(include ${catkin_INCLUDE_DIRS})

#find_package(Eigen REQUIRED)
#include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(JPEG REQUIRED)
include_directories(${JPEG_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})

#if you want to use multi-threading
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

#Only needed for the visualization
FIND_PACKAGE(MRPT REQUIRED gui)

#simple example of the particle filter usage
add_executable(mcl_example src/mcl_example.cpp)
target_link_libraries(mcl_example ${MRPT_LIBS} ${catkin_LIBRARIES})

#an example of 2d ndt-mcl
add_executable(2d_ndt_mcl_offline_test src/2d_ndt_mcl_offline_test.cpp src/CMyEllipsoid.cpp src/CParticleFilter.cpp)

find_package(Boost REQUIRED COMPONENTS signals)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(2d_ndt_mcl_offline_test ${Boost_LIBRARIES}  ${catkin_LIBRARIES})
target_link_libraries(2d_ndt_mcl_offline_test ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${MRPT_LIBS})

add_executable(2d_ndt_mcl_node src/2d_ndt_mcl_node.cpp src/CMyEllipsoid.cpp src/CParticleFilter.cpp)
target_link_libraries(2d_ndt_mcl_node ${Boost_LIBRARIES}  ${catkin_LIBRARIES})
target_link_libraries(2d_ndt_mcl_node ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${MRPT_LIBS})

## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)
# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
install(TARGETS 2d_ndt_mcl_node mcl_example 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )
install(FILES
   launch/2d_ndt_mcl_node.launch 
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY data/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
