ament_add_gtest(unit_tests main.cpp ekf_basic.cpp)
target_link_libraries(unit_tests state_estimation_sensor_sim ekf)
