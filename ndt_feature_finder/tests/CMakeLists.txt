add_executable(test_cell_orientation test_cell_orientation.cpp)
target_link_libraries(test_cell_orientation ndt_corner ${catkin_LIBRARIES})

add_executable(test_corner_detection test_corner_detection.cpp)
target_link_libraries(test_corner_detection ndt_corner ${catkin_LIBRARIES})

add_executable(test_corner_map test_corner_map.cpp)
target_link_libraries(test_corner_map ndt_corner ${catkin_LIBRARIES})

# add_executable(test_visu test_visu.cpp)
# target_link_libraries(test_visu ndtCorner ${catkin_LIBRARIES})

add_executable(test_corner_base test_corner_base.cpp)
target_link_libraries(test_corner_base ndt_corner ${catkin_LIBRARIES})

add_executable(conversionOpenCV conversionOpenCV.cpp)
target_link_libraries(conversionOpenCV ndt_corner ${OpenCV_LIBS}  ${catkin_LIBRARIES})

add_executable(test_ray_collision test_ray_collision.cpp)
target_link_libraries(test_ray_collision -lboost_unit_test_framework ${OpenCV_LIBS})
# target_link_libraries(tes ndtCorner ${OpenCV_LIBS}  ${catkin_LIBRARIES})

add_executable(test_estimation_cell test_estimation_cell.cpp)
target_link_libraries(test_estimation_cell ndt_corner ${catkin_LIBRARIES})

add_executable(corner_extraction corner_extraction.cpp)
target_link_libraries(corner_extraction ndt_corner ${catkin_LIBRARIES})
