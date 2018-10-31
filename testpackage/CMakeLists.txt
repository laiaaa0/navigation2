cmake_minimum_required(VERSION 3.5)
project(testpackage)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
)

set(executable_name testpackage)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)

# add_library(${library_name}
  # src/dwb_controller.cpp
# )

set(dependencies
  rclcpp
)

# ament_target_dependencies(${library_name}
#   ${dependencies}
# )

ament_target_dependencies(${executable_name}
  ${dependencies}
)
# target_link_libraries(${executable_name} ${library_name})

install(TARGETS ${executable_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY include/
  # DESTINATION include/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
