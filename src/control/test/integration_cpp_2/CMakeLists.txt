ament_add_gtest(test_dwa_controller_cpp2
  test_dwa_controller_cpp2.cpp
  ENV TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR})
ament_target_dependencies( test_dwa_controller_cpp2
  ${PROJECT_DEPENDENCIES}
)
target_link_libraries(test_dwa_controller_cpp2 stdc++fs)
