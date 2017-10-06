cmake_minimum_required(VERSION 2.8.3)
project(ndt_evaluation)

find_package(catkin REQUIRED COMPONENTS graph_map ndt_registration ndt_map ndt_generic tf tf_conversions pcl_ros)
find_package(Boost REQUIRED COMPONENTS program_options system)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
    DEPENDS eigen3 libpcl-all-dev
    CATKIN_DEPENDS ndt_registration ndt_map ndt_generic tf tf_conversions pcl_ros
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)



include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

set (${PROJECT_NAME}_LIB_SRCS
        src/registration_score_eval.cpp
     )
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(registration_score_evaluate src/registration_score_evaluate_main.cpp)
target_link_libraries(registration_score_evaluate ${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
