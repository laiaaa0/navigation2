cmake_minimum_required(VERSION 3.5)
project(nav2_random_tester)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)

nav2_package()

include_directories(
  include
)

add_subdirectory(src)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  find_package(ament_cmake_pytest)
  add_subdirectory(test)
endif()

ament_package()
