cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

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
rosbuild_add_executable(mcl_example src/mcl_example.cpp)
target_link_libraries(mcl_example ${MRPT_LIBS})

#an example of 2d ndt-mcl
rosbuild_add_executable(2d_ndt_mcl_offline_test src/2d_ndt_mcl_offline_test.cpp src/CMyEllipsoid.cpp src/CParticleFilter.cpp)
rosbuild_link_boost(2d_ndt_mcl_offline_test signals)
target_link_libraries(2d_ndt_mcl_offline_test ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${MRPT_LIBS})

rosbuild_add_executable(2d_ndt_mcl_node src/2d_ndt_mcl_node.cpp src/CMyEllipsoid.cpp src/CParticleFilter.cpp)
rosbuild_link_boost(2d_ndt_mcl_node signals)
target_link_libraries(2d_ndt_mcl_node ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${MRPT_LIBS})

